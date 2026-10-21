//! Garland-Heckbert error quadrics (area-weighted).

use amg_core::Point3;

/// Symmetric 4x4 quadric `[A b; b' c]` plus the total face area `w` that went into it.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Quadric {
    /// Upper triangle of `A`: `a11 a12 a13 a22 a23 a33`.
    a: [f64; 6],
    b: [f64; 3],
    c: f64,
    /// Sum of the face areas (not the penalty weights); normalises the error to mm.
    pub(super) w: f64,
}

impl Quadric {
    /// Squared distance to the plane `n . p + d = 0` (`n` unit), scaled by `weight`.
    pub(super) fn plane(n: Point3, d: f64, weight: f64) -> Quadric {
        Quadric {
            a: [
                n.x * n.x * weight,
                n.x * n.y * weight,
                n.x * n.z * weight,
                n.y * n.y * weight,
                n.y * n.z * weight,
                n.z * n.z * weight,
            ],
            b: [n.x * d * weight, n.y * d * weight, n.z * d * weight],
            c: d * d * weight,
            w: 0.0,
        }
    }

    pub(super) fn add(&mut self, o: &Quadric) {
        for i in 0..6 {
            self.a[i] += o.a[i];
        }
        for i in 0..3 {
            self.b[i] += o.b[i];
        }
        self.c += o.c;
        self.w += o.w;
    }

    pub(super) fn sum(&self, o: &Quadric) -> Quadric {
        let mut q = *self;
        q.add(o);
        q
    }

    /// `p' A p + 2 b' p + c`, clamped at 0.
    pub(super) fn eval(&self, p: Point3) -> f64 {
        let a = &self.a;
        let v = a[0] * p.x * p.x
            + a[3] * p.y * p.y
            + a[5] * p.z * p.z
            + 2.0 * (a[1] * p.x * p.y + a[2] * p.x * p.z + a[4] * p.y * p.z)
            + 2.0 * (self.b[0] * p.x + self.b[1] * p.y + self.b[2] * p.z)
            + self.c;
        v.max(0.0)
    }

    /// The minimiser of the quadric, or `None` when `A` is (nearly) singular.
    pub(super) fn optimum(&self) -> Option<Point3> {
        let [a, b, c, d, e, f] = self.a;
        // Cofactors of the symmetric matrix [[a b c] [b d e] [c e f]].
        let c00 = d * f - e * e;
        let c01 = c * e - b * f;
        let c02 = b * e - c * d;
        let det = a * c00 + b * c01 + c * c02;
        let scale = (a + d + f) / 3.0;
        if scale <= 0.0 || det.abs() <= 1e-6 * scale * scale * scale {
            return None;
        }
        let c11 = a * f - c * c;
        let c12 = b * c - a * e;
        let c22 = a * d - b * b;
        let inv = 1.0 / det;
        let [x, y, z] = self.b;
        let p = Point3::new(
            -(c00 * x + c01 * y + c02 * z) * inv,
            -(c01 * x + c11 * y + c12 * z) * inv,
            -(c02 * x + c12 * y + c22 * z) * inv,
        );
        p.is_finite().then_some(p)
    }
}
