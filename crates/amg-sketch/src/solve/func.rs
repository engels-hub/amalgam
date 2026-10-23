//! Scalar geometric primitives with hand-written analytic gradients.
//!
//! Every residual row is a linear combination of these terms (plus a constant), so the
//! chain rule lives here once. Each primitive reads `arity()` consecutive local values and
//! writes the same number of partial derivatives.
//!
//! Degenerate inputs (zero-length direction, point on a centre) return a value of 0 and a zero
//! gradient instead of NaN; the solver's rank analysis (SK-04) reports them.

use std::f64::consts::TAU;

/// How a 2D direction vector is formed from four local values.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Dir {
    /// `[ax, ay, bx, by]` → `b − a` (a line's direction of travel `end − start`).
    Line,
    /// `[cx, cy, px, py]` → `perp(p − c) = (−(p−c).y, (p−c).x)`, the CCW tangent of a circle
    /// about `c` at `p`.
    ArcTangent,
}

impl Dir {
    fn vector(self, v: &[f64]) -> (f64, f64) {
        match self {
            Dir::Line => (v[2] - v[0], v[3] - v[1]),
            Dir::ArcTangent => (v[1] - v[3], v[2] - v[0]),
        }
    }

    /// Pulls a gradient with respect to the vector back to the four local values.
    fn pull_back(self, gx: f64, gy: f64, g: &mut [f64]) {
        match self {
            Dir::Line => {
                g[0] = -gx;
                g[1] = -gy;
                g[2] = gx;
                g[3] = gy;
            }
            Dir::ArcTangent => {
                // ux = cy − py, uy = px − cx
                g[0] = -gy;
                g[1] = gx;
                g[2] = gy;
                g[3] = -gx;
            }
        }
    }
}

/// How a radius is read: a radius param (circle) or `|s − c|` from four values (arc).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Rad {
    /// `[r]`.
    Param,
    /// `[sx, sy, cx, cy]` → `|s − c|`.
    Dist,
}

impl Rad {
    fn arity(self) -> usize {
        match self {
            Rad::Param => 1,
            Rad::Dist => 4,
        }
    }

    fn eval(self, v: &[f64], g: &mut [f64]) -> f64 {
        match self {
            Rad::Param => {
                g[0] = 1.0;
                v[0]
            }
            Rad::Dist => dist(v, g),
        }
    }
}

/// A scalar function of `arity()` local values. Points are `(x, y)` pairs.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Func {
    /// `[x]` → `x`.
    Coord,
    /// `[p, q]` → `|p − q|`.
    Dist,
    /// `[r1.., r2..]` → `|r1 − r2|` for two radii read per [`Rad`].
    AbsRadDiff(Rad, Rad),
    /// `[a, b, p]` → `cross(b − a, p − a) / |b − a|`: signed distance of `p` from the line
    /// `a → b`, positive on the left.
    SignedDist,
    /// `[a, b, p, q]` → signed distance of the midpoint `(p + q)/2` from the line `a → b`.
    MidSignedDist,
    /// `[a, b, p, q]` → `dot(q − p, (b − a)/|b − a|)`.
    DirDot,
    /// `[u; 4, w; 4]` (both `Dir::Line`) → `cross(u, w) / (|u||w|)` (sine of the angle).
    Sin,
    /// `[u; 4, w; 4]` (both `Dir::Line`) → `dot(u, w) / (|u||w|)` (cosine of the angle).
    Cos,
    /// `[u; 4, w; 4]` → `atan2(cross(u, w), dot(u, w))` ∈ (−π, π], the signed angle from `u`
    /// to `w`.
    Angle(Dir, Dir),
    /// `[c, s, e]` → `|s − c| · sweep`, `sweep` = CCW angle from `s − c` to `e − c` in
    /// (0, 2π] (the arc length of the CCW arc).
    ArcLen,
    /// `[p, c, M, b, t]` (`px, py, cx, cy, Mx, My, b, t`) → component `k` (0 = x, 1 = y) of
    /// `p − P(t)`, where `P(t) = c + m·cos t + perp(m)·(b/|m|)·sin t` and `m = M − c` (SK-22).
    EllipseOn(usize),
    /// `[A, B, c, M, b]` (`Ax, Ay, Bx, By, cx, cy, Mx, My, b`) → half-width `h` of the ellipse
    /// with centre `c`, major end `M` and minor radius `b` in the direction normal to the line
    /// `A → B`: `h² = b² + (1 − b²/a²)·q²`, `q = cross(B − A, M − c)/|B − A|`, `a = |M − c|`.
    /// The line is tangent to the ellipse when the signed centre distance equals `±h`.
    EllipseSupport,
}

/// Largest arity of any [`Func`].
pub(crate) const MAX_ARITY: usize = 9;

impl Func {
    pub(crate) fn arity(self) -> usize {
        match self {
            Func::Coord => 1,
            Func::AbsRadDiff(a, b) => a.arity() + b.arity(),
            Func::Dist => 4,
            Func::SignedDist | Func::ArcLen => 6,
            Func::MidSignedDist | Func::DirDot | Func::Sin | Func::Cos | Func::Angle(..) => 8,
            Func::EllipseOn(_) => 8,
            Func::EllipseSupport => 9,
        }
    }

    /// Value at `v[..arity]`; writes `∂f/∂v` to `g[..arity]`.
    pub(crate) fn eval(self, v: &[f64], g: &mut [f64]) -> f64 {
        match self {
            Func::Coord => {
                g[0] = 1.0;
                v[0]
            }
            Func::Dist => dist(v, g),
            Func::AbsRadDiff(ka, kb) => {
                let na = ka.arity();
                let nb = kb.arity();
                let ra = ka.eval(&v[..na], &mut g[..na]);
                let rb = kb.eval(&v[na..na + nb], &mut g[na..na + nb]);
                // f = s·(ra − rb), s = sign(ra − rb) (+1 at the kink)
                let d = ra - rb;
                let s = if d >= 0.0 { 1.0 } else { -1.0 };
                g[..na].iter_mut().for_each(|x| *x *= s);
                g[na..na + nb].iter_mut().for_each(|x| *x *= -s);
                d.abs()
            }
            Func::SignedDist => signed_dist(v, g),
            Func::MidSignedDist => {
                let local = [
                    v[0],
                    v[1],
                    v[2],
                    v[3],
                    0.5 * (v[4] + v[6]),
                    0.5 * (v[5] + v[7]),
                ];
                let mut gl = [0.0; 6];
                let f = signed_dist(&local, &mut gl);
                g[..4].copy_from_slice(&gl[..4]);
                g[4] = 0.5 * gl[4];
                g[5] = 0.5 * gl[5];
                g[6] = 0.5 * gl[4];
                g[7] = 0.5 * gl[5];
                f
            }
            Func::DirDot => dir_dot(v, g),
            Func::Sin => sin_cos(v, g, true),
            Func::Cos => sin_cos(v, g, false),
            Func::Angle(du, dw) => angle(du, dw, v, g),
            Func::ArcLen => arc_len(v, g),
            Func::EllipseOn(k) => ellipse_on(k, v, g),
            Func::EllipseSupport => ellipse_support(v, g),
        }
    }
}

/// Component `k` of `p − P(t)`; see [`Func::EllipseOn`].
fn ellipse_on(k: usize, v: &[f64], g: &mut [f64]) -> f64 {
    let (px, py, cx, cy, ex, ey, b, t) = (v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
    let (mx, my) = (ex - cx, ey - cy);
    let a2 = mx * mx + my * my;
    if a2 <= 0.0 {
        g[..8].fill(0.0);
        return 0.0;
    }
    let a = a2.sqrt();
    let (cs, sn) = (t.cos(), t.sin());
    let kk = b / a;
    // w = perp(m)·k = (−my·k, mx·k)
    let (f, gmx, gmy, gb, gt) = if k == 0 {
        (
            px - cx - mx * cs + my * kk * sn,
            -cs - b * mx * my * sn / (a2 * a),
            sn * (kk - b * my * my / (a2 * a)),
            my * sn / a,
            mx * sn + my * kk * cs,
        )
    } else {
        (
            py - cy - my * cs - mx * kk * sn,
            -sn * (kk - b * mx * mx / (a2 * a)),
            -cs + b * mx * my * sn / (a2 * a),
            -mx * sn / a,
            my * sn - mx * kk * cs,
        )
    };
    let (dpx, dpy) = if k == 0 { (1.0, 0.0) } else { (0.0, 1.0) };
    // c appears directly (−1) and through m = M − c.
    g[0] = dpx;
    g[1] = dpy;
    g[2] = -dpx - gmx;
    g[3] = -dpy - gmy;
    g[4] = gmx;
    g[5] = gmy;
    g[6] = gb;
    g[7] = gt;
    f
}

/// See [`Func::EllipseSupport`].
fn ellipse_support(v: &[f64], g: &mut [f64]) -> f64 {
    let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
    let (mx, my) = (v[6] - v[4], v[7] - v[5]);
    let b = v[8];
    let l2 = dx * dx + dy * dy;
    let a2 = mx * mx + my * my;
    if l2 <= 0.0 || a2 <= 0.0 {
        g[..9].fill(0.0);
        return 0.0;
    }
    let l = l2.sqrt();
    let q = (dx * my - dy * mx) / l;
    let h2 = b * b + q * q * (1.0 - b * b / a2);
    if h2 <= 0.0 {
        g[..9].fill(0.0);
        return 0.0;
    }
    let h = h2.sqrt();
    let dh2_db = 2.0 * b * (1.0 - q * q / a2);
    let dh2_dq = 2.0 * q * (1.0 - b * b / a2);
    let dh2_da2 = q * q * b * b / (a2 * a2);
    let (dq_ddx, dq_ddy) = (my / l - q * dx / l2, -mx / l - q * dy / l2);
    let (dq_dmx, dq_dmy) = (-dy / l, dx / l);
    let s = 0.5 / h;
    let gdx = s * dh2_dq * dq_ddx;
    let gdy = s * dh2_dq * dq_ddy;
    let gmx = s * (dh2_dq * dq_dmx + dh2_da2 * 2.0 * mx);
    let gmy = s * (dh2_dq * dq_dmy + dh2_da2 * 2.0 * my);
    g[0] = -gdx;
    g[1] = -gdy;
    g[2] = gdx;
    g[3] = gdy;
    g[4] = -gmx;
    g[5] = -gmy;
    g[6] = gmx;
    g[7] = gmy;
    g[8] = s * dh2_db;
    h
}

/// Wraps an angle to (−π, π].
pub(crate) fn wrap_pi(a: f64) -> f64 {
    let w = a.rem_euclid(TAU);
    if w > std::f64::consts::PI { w - TAU } else { w }
}

fn dist(v: &[f64], g: &mut [f64]) -> f64 {
    let (dx, dy) = (v[0] - v[2], v[1] - v[3]);
    let l = dx.hypot(dy);
    if l > 0.0 {
        let (ux, uy) = (dx / l, dy / l);
        g[..4].copy_from_slice(&[ux, uy, -ux, -uy]);
    } else {
        g[..4].fill(0.0);
    }
    l
}

fn signed_dist(v: &[f64], g: &mut [f64]) -> f64 {
    let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
    let (wx, wy) = (v[4] - v[0], v[5] - v[1]);
    let l2 = dx * dx + dy * dy;
    if l2 <= 0.0 {
        g[..6].fill(0.0);
        return 0.0;
    }
    let l = l2.sqrt();
    let c = dx * wy - dy * wx;
    let f = c / l;
    // ∂f/∂d = (wy, −wx)/l − f·d/l²;  ∂f/∂w = (−dy, dx)/l
    let gdx = wy / l - f * dx / l2;
    let gdy = -wx / l - f * dy / l2;
    let gwx = -dy / l;
    let gwy = dx / l;
    // d = b − a, w = p − a
    g[0] = -gdx - gwx;
    g[1] = -gdy - gwy;
    g[2] = gdx;
    g[3] = gdy;
    g[4] = gwx;
    g[5] = gwy;
    f
}

fn dir_dot(v: &[f64], g: &mut [f64]) -> f64 {
    let (dx, dy) = (v[2] - v[0], v[3] - v[1]);
    let (qx, qy) = (v[6] - v[4], v[7] - v[5]);
    let l2 = dx * dx + dy * dy;
    if l2 <= 0.0 {
        g[..8].fill(0.0);
        return 0.0;
    }
    let l = l2.sqrt();
    let f = (qx * dx + qy * dy) / l;
    // ∂f/∂d = (q−p)/l − f·d/l²;  ∂f/∂(q−p) = d/l
    let gdx = qx / l - f * dx / l2;
    let gdy = qy / l - f * dy / l2;
    let (gqx, gqy) = (dx / l, dy / l);
    g[..8].copy_from_slice(&[-gdx, -gdy, gdx, gdy, -gqx, -gqy, gqx, gqy]);
    f
}

fn sin_cos(v: &[f64], g: &mut [f64], sine: bool) -> f64 {
    let (ux, uy) = Dir::Line.vector(&v[..4]);
    let (wx, wy) = Dir::Line.vector(&v[4..8]);
    let nu2 = ux * ux + uy * uy;
    let nw2 = wx * wx + wy * wy;
    if nu2 <= 0.0 || nw2 <= 0.0 {
        g[..8].fill(0.0);
        return 0.0;
    }
    let inv = 1.0 / (nu2 * nw2).sqrt();
    let (num, gnu, gnw) = if sine {
        // cross(u, w): ∂/∂u = (wy, −wx), ∂/∂w = (−uy, ux)
        (ux * wy - uy * wx, (wy, -wx), (-uy, ux))
    } else {
        (ux * wx + uy * wy, (wx, wy), (ux, uy))
    };
    let f = num * inv;
    let gux = gnu.0 * inv - f * ux / nu2;
    let guy = gnu.1 * inv - f * uy / nu2;
    let gwx = gnw.0 * inv - f * wx / nw2;
    let gwy = gnw.1 * inv - f * wy / nw2;
    let (gu, gw) = g[..8].split_at_mut(4);
    Dir::Line.pull_back(gux, guy, gu);
    Dir::Line.pull_back(gwx, gwy, gw);
    f
}

fn angle(du: Dir, dw: Dir, v: &[f64], g: &mut [f64]) -> f64 {
    let (ux, uy) = du.vector(&v[..4]);
    let (wx, wy) = dw.vector(&v[4..8]);
    let c = ux * wy - uy * wx;
    let d = ux * wx + uy * wy;
    let n2 = c * c + d * d; // = |u|²|w|²
    if n2 <= 0.0 {
        g[..8].fill(0.0);
        return 0.0;
    }
    // dθ = (d·dc − c·dd) / (c² + d²)
    let gux = (d * wy - c * wx) / n2;
    let guy = (-d * wx - c * wy) / n2;
    let gwx = (-d * uy - c * ux) / n2;
    let gwy = (d * ux - c * uy) / n2;
    let (gu, gw) = g[..8].split_at_mut(4);
    du.pull_back(gux, guy, gu);
    dw.pull_back(gwx, gwy, gw);
    c.atan2(d)
}

/// CCW sweep from `s − c` to `e − c` in (0, 2π].
pub(crate) fn sweep(c: (f64, f64), s: (f64, f64), e: (f64, f64)) -> f64 {
    let ts = (s.1 - c.1).atan2(s.0 - c.0);
    let te = (e.1 - c.1).atan2(e.0 - c.0);
    let w = (te - ts).rem_euclid(TAU);
    if w <= 0.0 { TAU } else { w }
}

fn arc_len(v: &[f64], g: &mut [f64]) -> f64 {
    let (sx, sy) = (v[2] - v[0], v[3] - v[1]);
    let (ex, ey) = (v[4] - v[0], v[5] - v[1]);
    let rs2 = sx * sx + sy * sy;
    let re2 = ex * ex + ey * ey;
    if rs2 <= 0.0 || re2 <= 0.0 {
        g[..6].fill(0.0);
        return 0.0;
    }
    let r = rs2.sqrt();
    let w = sweep((v[0], v[1]), (v[2], v[3]), (v[4], v[5]));
    // ∂r/∂s = (s−c)/r; ∂θ_p/∂p = (−(p−c).y, (p−c).x)/|p−c|²; sweep = θe − θs (mod 2π)
    let (grx, gry) = (sx / r, sy / r);
    let (gsx, gsy) = (sy / rs2, -sx / rs2); // ∂sweep/∂s = −∂θs/∂s
    let (gex, gey) = (-ey / re2, ex / re2); // ∂sweep/∂e = ∂θe/∂e
    let gs = (w * grx + r * gsx, w * gry + r * gsy);
    let ge = (r * gex, r * gey);
    g[..6].copy_from_slice(&[-gs.0 - ge.0, -gs.1 - ge.1, gs.0, gs.1, ge.0, ge.1]);
    r * w
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Func; 19] = [
        Func::EllipseOn(0),
        Func::EllipseOn(1),
        Func::EllipseSupport,
        Func::Coord,
        Func::AbsRadDiff(Rad::Param, Rad::Param),
        Func::AbsRadDiff(Rad::Param, Rad::Dist),
        Func::AbsRadDiff(Rad::Dist, Rad::Param),
        Func::AbsRadDiff(Rad::Dist, Rad::Dist),
        Func::Dist,
        Func::SignedDist,
        Func::MidSignedDist,
        Func::DirDot,
        Func::Sin,
        Func::Cos,
        Func::Angle(Dir::Line, Dir::Line),
        Func::Angle(Dir::Line, Dir::ArcTangent),
        Func::Angle(Dir::ArcTangent, Dir::Line),
        Func::Angle(Dir::ArcTangent, Dir::ArcTangent),
        Func::ArcLen,
    ];

    #[test]
    fn gradients_match_central_differences() {
        let base = [1.3, -0.7, 4.1, 2.2, -2.5, 3.9, 0.4, -3.3, 1.7];
        for f in ALL {
            let n = f.arity();
            let mut g = [0.0; MAX_ARITY];
            f.eval(&base, &mut g);
            for k in 0..n {
                let h = 1e-6;
                let mut p = base;
                let mut m = base;
                p[k] += h;
                m[k] -= h;
                let mut scratch = [0.0; MAX_ARITY];
                let fd = (f.eval(&p, &mut scratch) - f.eval(&m, &mut scratch)) / (2.0 * h);
                assert!((fd - g[k]).abs() < 1e-7, "{f:?} slot {k}: {fd} vs {}", g[k]);
            }
        }
    }

    #[test]
    fn degenerate_inputs_give_zero_not_nan() {
        let zero = [0.0; MAX_ARITY];
        for f in ALL {
            let mut g = [f64::NAN; MAX_ARITY];
            let v = f.eval(&zero, &mut g);
            assert!(v.is_finite(), "{f:?}");
            assert!(g[..f.arity()].iter().all(|x| x.is_finite()), "{f:?}");
        }
    }

    #[test]
    fn conventions() {
        let mut g = [0.0; MAX_ARITY];
        // Point (0, 1) is left of the x axis.
        assert!(
            (Func::SignedDist.eval(&[0.0, 0.0, 2.0, 0.0, 0.0, 1.0], &mut g) - 1.0).abs() < 1e-15
        );
        // Angle from +x to +y is +π/2.
        let a = Func::Angle(Dir::Line, Dir::Line)
            .eval(&[0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], &mut g);
        assert!((a - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
        // CCW tangent at (1, 0) about the origin is +y.
        let a = Func::Angle(Dir::Line, Dir::ArcTangent)
            .eval(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0], &mut g);
        assert!(a.abs() < 1e-15);
        // Quarter arc of radius 2.
        let l = Func::ArcLen.eval(&[0.0, 0.0, 2.0, 0.0, 0.0, 2.0], &mut g);
        assert!((l - std::f64::consts::PI).abs() < 1e-15);
        assert!((wrap_pi(-std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-15);
        assert!(
            (wrap_pi(3.0 * std::f64::consts::PI / 2.0) + std::f64::consts::FRAC_PI_2).abs() < 1e-15
        );
    }
}
