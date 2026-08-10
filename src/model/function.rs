use bevy::prelude::*;

/// Mathematical functions mapping a scalar coordinate to a scalar value.
///
/// Used by joints (SpatialTransform axes), muscles (force-length/velocity curves),
/// coordinate couplers, and prescribed motion.
///
/// All variants support `evaluate(q)` and `derivative()` for Jacobian computation.
#[derive(Component, Clone, Debug)]
pub enum Function {
    /// f(q) = c — constant value.
    Constant(f64),
    /// f(q) = a0 + a1*q — linear (slope, intercept).
    Linear { slope: f64, intercept: f64 },
    /// f(q) = a0 + a1*q + a2*q^2 + ... — global polynomial.
    Polynomial(Vec<f64>),
    /// Piecewise linear interpolation between (x, y) knot pairs.
    /// x values must be sorted ascending. Clamps outside range.
    PiecewiseLinear { x: Vec<f64>, y: Vec<f64> },
    /// Piecewise cubic spline (OpenSim SimmSpline / natural cubic spline).
    /// x values must be sorted ascending. Clamps outside range.
    CubicSpline { x: Vec<f64>, y: Vec<f64> },
    /// Analytical derivative of a cubic spline (piecewise quadratic).
    /// Stored as coefficients (b, 2c, 3d) per interval from the original
    /// cubic S_i(x) = a + b*(x-x_i) + c*(x-x_i)^2 + d*(x-x_i)^3.
    CubicSplineDeriv {
        x: Vec<f64>,
        /// (b, 2c, 3d) per interval — coefficients of the quadratic derivative.
        coeffs: Vec<(f64, f64, f64)>,
    },
}

impl Function {
    /// Evaluate f(q).
    pub fn evaluate(&self, q: f64) -> f64 {
        match self {
            Function::Constant(c) => *c,
            Function::Linear { slope, intercept } => intercept + slope * q,
            Function::Polynomial(coeffs) => {
                coeffs.iter().enumerate().fold(0.0, |acc, (i, &c)| {
                    acc + c * q.powi(i as i32)
                })
            }
            Function::PiecewiseLinear { x, y } => interpolate_linear(x, y, q),
            Function::CubicSpline { x, y } => interpolate_cubic_spline(x, y, q),
            Function::CubicSplineDeriv { x, coeffs } => {
                if x.is_empty() || coeffs.is_empty() {
                    return 0.0;
                }
                let q = q.clamp(x[0], x[x.len() - 1]);
                let i = x.iter().position(|&xi| xi > q).unwrap_or(x.len()) - 1;
                let i = i.min(coeffs.len() - 1);
                let dx = q - x[i];
                let (b, c2, d3) = coeffs[i];
                b + c2 * dx + d3 * dx * dx
            }
        }
    }

    /// Compute the derivative f'(q).
    pub fn derivative(&self) -> Function {
        match self {
            Function::Constant(_) => Function::Constant(0.0),
            Function::Linear { slope, .. } => Function::Constant(*slope),
            Function::Polynomial(coeffs) => {
                Function::Polynomial(
                    coeffs.iter().enumerate().skip(1)
                        .map(|(i, &c)| c * i as f64)
                        .collect(),
                )
            }
            // Numerical derivative for splines — use central difference.
            // For analytical derivative, use CubicSpline::derivative() directly.
            Function::PiecewiseLinear { x, y } => {
                // Derivative of piecewise linear is piecewise constant.
                // Return a PiecewiseLinear with midpoints and slopes.
                if x.len() < 2 {
                    return Function::Constant(0.0);
                }
                let dx: Vec<f64> = x.windows(2).map(|w| (w[0] + w[1]) / 2.0).collect();
                let dy: Vec<f64> = x.windows(2)
                    .zip(y.windows(2))
                    .map(|(xn, yn)| (yn[1] - yn[0]) / (xn[1] - xn[0]))
                    .collect();
                Function::PiecewiseLinear { x: dx, y: dy }
            }
            Function::CubicSpline { x, y } => {
                let coeffs = cubic_spline_coefficients(x, y);
                let deriv_coeffs: Vec<(f64, f64, f64)> = coeffs.iter()
                    .map(|&(_a, b, c, d)| (b, 2.0 * c, 3.0 * d))
                    .collect();
                Function::CubicSplineDeriv {
                    x: x.clone(),
                    coeffs: deriv_coeffs,
                }
            }
            Function::CubicSplineDeriv { x, coeffs } => {
                // Derivative of piecewise quadratic (b + c2*dx + d3*dx^2)
                // is piecewise linear (c2 + 2*d3*dx)
                if x.len() < 2 || coeffs.is_empty() {
                    return Function::Constant(0.0);
                }
                let mid_x: Vec<f64> = x.windows(2).map(|w| (w[0] + w[1]) / 2.0).collect();
                let mid_y: Vec<f64> = coeffs.iter()
                    .map(|&(_b, c2, _d3)| c2) // value at dx=0 (i.e. at knot)
                    .collect();
                Function::PiecewiseLinear { x: mid_x, y: mid_y }
            }
        }
    }
}



// ── Interpolation helpers ─────────────────────────────

fn interpolate_linear(x: &[f64], y: &[f64], q: f64) -> f64 {
    if x.is_empty() {
        return 0.0;
    }
    if x.len() == 1 {
        return y[0];
    }

    // Clamp to range
    if q <= x[0] {
        return y[0];
    }
    if q >= x[x.len() - 1] {
        return y[y.len() - 1];
    }

    // Find bracketing interval
    let idx = x.iter().position(|&xi| xi > q).unwrap_or(x.len()) - 1;
    let t = (q - x[idx]) / (x[idx + 1] - x[idx]);
    y[idx] + t * (y[idx + 1] - y[idx])
}

/// Natural cubic spline interpolation.
///
/// Solves for second derivatives at each knot via tridiagonal system,
/// then evaluates the cubic polynomial in the bracketing interval.
fn interpolate_cubic_spline(x: &[f64], y: &[f64], q: f64) -> f64 {
    let n = x.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 {
        return y[0];
    }
    if n == 2 {
        return interpolate_linear(x, y, q);
    }

    // Compute second derivatives (natural spline: M[0] = M[n-1] = 0)
    let m = cubic_spline_second_derivatives(x, y);

    // Clamp
    let q = if q <= x[0] { return y[0] } else if q >= x[n - 1] { return y[n - 1] } else { q };

    // Find interval
    let i = x.iter().position(|&xi| xi > q).unwrap_or(n) - 1;
    let h = x[i + 1] - x[i];
    let a = (x[i + 1] - q) / h;
    let b = (q - x[i]) / h;

    a * y[i] + b * y[i + 1]
        + ((a * a * a - a) * m[i] + (b * b * b - b) * m[i + 1]) * h * h / 6.0
}

/// Solve for second derivatives of a natural cubic spline.
fn cubic_spline_second_derivatives(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut m = vec![0.0; n];
    if n < 3 {
        return m;
    }

    // Tridiagonal system for natural spline
    let mut a = vec![0.0; n];
    let mut b = vec![2.0; n]; // diagonal
    let mut c = vec![0.0; n];
    let mut d = vec![0.0; n];

    for i in 1..n - 1 {
        let h_prev = x[i] - x[i - 1];
        let h_next = x[i + 1] - x[i];
        a[i] = h_prev;
        b[i] = 2.0 * (h_prev + h_next);
        c[i] = h_next;
        d[i] = 6.0 * ((y[i + 1] - y[i]) / h_next - (y[i] - y[i - 1]) / h_prev);
    }

    // Thomas algorithm (forward sweep)
    for i in 1..n - 1 {
        let w = a[i] / b[i - 1];
        b[i] -= w * c[i - 1];
        d[i] -= w * d[i - 1];
    }

    // Back substitution
    m[n - 2] = d[n - 2] / b[n - 2];
    for i in (1..n - 2).rev() {
        m[i] = (d[i] - c[i] * m[i + 1]) / b[i];
    }

    m
}

/// Compute full cubic spline coefficients for derivative support.
/// Returns (a, b, c, d) per interval where:
///   S_i(x) = a + b*(x-x_i) + c*(x-x_i)^2 + d*(x-x_i)^3
pub fn cubic_spline_coefficients(x: &[f64], y: &[f64]) -> Vec<(f64, f64, f64, f64)> {
    let n = x.len();
    let m = cubic_spline_second_derivatives(x, y);
    let mut coeffs = Vec::with_capacity(n.saturating_sub(1));

    for i in 0..n - 1 {
        let h = x[i + 1] - x[i];
        let a = y[i];
        let b = (y[i + 1] - y[i]) / h - h * (2.0 * m[i] + m[i + 1]) / 6.0;
        let c = m[i] / 2.0;
        let d = (m[i + 1] - m[i]) / (6.0 * h);
        coeffs.push((a, b, c, d));
    }

    coeffs
}

// ── Tests ─────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant() {
        let f = Function::Constant(5.0);
        assert_eq!(f.evaluate(100.0), 5.0);
        assert_eq!(f.derivative().evaluate(100.0), 0.0);
    }

    #[test]
    fn test_linear() {
        let f = Function::Linear { slope: 2.0, intercept: 3.0 };
        assert!((f.evaluate(4.0) - 11.0).abs() < 1e-12);
        assert!((f.derivative().evaluate(4.0) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_polynomial() {
        // f(q) = 1 + 2q + 3q^2
        let f = Function::Polynomial(vec![1.0, 2.0, 3.0]);
        assert!((f.evaluate(2.0) - 17.0).abs() < 1e-12); // 1 + 4 + 12
        // f'(q) = 2 + 6q
        let fp = f.derivative();
        assert!((fp.evaluate(2.0) - 14.0).abs() < 1e-12);
    }

    #[test]
    fn test_piecewise_linear() {
        let f = Function::PiecewiseLinear {
            x: vec![0.0, 1.0, 2.0],
            y: vec![0.0, 10.0, 0.0],
        };
        assert!((f.evaluate(0.5) - 5.0).abs() < 1e-12);
        assert!((f.evaluate(1.5) - 5.0).abs() < 1e-12);
        assert!((f.evaluate(-1.0) - 0.0).abs() < 1e-12); // clamp
        assert!((f.evaluate(3.0) - 0.0).abs() < 1e-12); // clamp
    }

    #[test]
    fn test_cubic_spline_identity() {
        // f(x) = x on [0, 1, 2, 3]
        let f = Function::CubicSpline {
            x: vec![0.0, 1.0, 2.0, 3.0],
            y: vec![0.0, 1.0, 2.0, 3.0],
        };
        for &q in &[0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0] {
            assert!((f.evaluate(q) - q).abs() < 1e-10, "failed at q={q}");
        }
    }

    #[test]
    fn test_simm_spline_knee() {
        // OpenSim walker_knee_r rotation2: x in [0, 2.0944], y varies
        let x: Vec<f64> = vec![0.0, 0.1745, 0.3491, 0.5236, 0.6981, 0.8727,
                               1.0472, 1.2217, 1.3963, 1.5708, 1.7453, 1.9199, 2.0944];
        let y: Vec<f64> = vec![0.0, 0.0127, 0.0227, 0.0296, 0.0332, 0.0335,
                               0.0309, 0.0258, 0.0189, 0.0114, 0.0044, -0.0005, -0.0017];
        let f = Function::CubicSpline { x: x.clone(), y: y.clone() };

        // Should match knots exactly
        for (i, &xi) in x.iter().enumerate() {
            assert!((f.evaluate(xi) - y[i]).abs() < 1e-10, "mismatch at knot {i}");
        }

        // Should interpolate smoothly between knots
        let mid = (x[3] + x[4]) / 2.0;
        let val = f.evaluate(mid);
        assert!(val > y[3].min(y[4]) && val < y[3].max(y[4]), "out of range at midpoint");
    }
}
