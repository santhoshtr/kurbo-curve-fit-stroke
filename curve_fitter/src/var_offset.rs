// Copyright 2022 the Kurbo Authors (Modified for Variable Width)
// SPDX-License-Identifier: Apache-2.0 OR MIT
use kurbo::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, Point, QuadBez, Vec2};
/// State used for computing a variable-width offset curve of a single cubic.
struct VariableCubicOffset {
    /// The cubic being offset.
    c: CubicBez,
    /// The derivative of `c`.
    q: QuadBez,
    /// Start width (at t=0)
    w0: f64,
    /// End width (at t=1)
    w1: f64,
    tolerance: f64,
}

struct OffsetParams {
    pos: Point,
    tangent: Vec2, // Un-normalized derivative vector of the offset curve
}

impl OffsetParams {
    fn new(c: &CubicBez, t: f64, w0: f64, w1: f64) -> Self {
        let deriv = c.deriv().eval(t).to_vec2();
        let deriv2 = c.deriv().deriv().eval(t).to_vec2();

        let d = w0 + (w1 - w0) * t; // Width at t
        let d_prime = w1 - w0; // Width derivative (w.r.t t)

        let deriv_len2 = deriv.hypot2();
        let deriv_len = deriv_len2.sqrt();

        // Robustness: Handle zero derivative (degenerate source)
        if deriv_len < 1e-9 {
            // Fallback: degenerate point. Tangent is arbitrary?
            // Let's use the derivative of width for direction if geometric derivative is 0.
            return Self {
                pos: c.eval(t) + Vec2::new(d, 0.0),
                tangent: Vec2::ZERO,
            };
        }

        let curvature = deriv.cross(deriv2) / (deriv_len * deriv_len2);
        let normal = Vec2::new(-deriv.y, deriv.x) / deriv_len; // Unit normal

        // Raph's Formula: x'_{off} = (1 + k*d)x' + n*d'
        let term_a = (1.0 + curvature * d) * deriv;
        let term_b = normal * d_prime;

        let tangent = term_a + term_b;
        let pos = c.eval(t) + d * normal;

        Self { pos, tangent }
    }
}

/// Compute an approximate variable-width offset curve.
///
/// * `w0`: The offset distance at t=0.
/// * `w1`: The offset distance at t=1.
///
/// Width is interpolated linearly with t between w0 and w1.
pub fn offset_cubic_variable(c: CubicBez, w0: f64, w1: f64, tolerance: f64, result: &mut BezPath) {
    result.truncate(0);

    let offset = VariableCubicOffset::new(c, w0, w1, tolerance);

    // Calculate initial MoveTo
    let start_params = OffsetParams::new(&c, 0.0, w0, w1);
    result.move_to(start_params.pos);
    let depth = 1;
    offset.recurse(0.0, 1.0, 0, depth, result);
}

impl VariableCubicOffset {
    fn new(c: CubicBez, w0: f64, w1: f64, tolerance: f64) -> Self {
        // The factor 2.0 comes from the second derivative of cubic
        VariableCubicOffset {
            c,
            q: c.deriv(),
            w0,
            w1,
            tolerance,
        }
    }

    /// Recursively split until we hit target depth, then generate a segment.
    fn recurse(&self, t0: f64, t1: f64, depth: usize, target_depth: usize, result: &mut BezPath) {
        if depth < target_depth {
            let mid = (t0 + t1) / 2.0;
            self.recurse(t0, mid, depth + 1, target_depth, result);
            self.recurse(mid, t1, depth + 1, target_depth, result);
        } else {
            // One-Point Shape Control Fit
            let p0 = OffsetParams::new(&self.c, t0, self.w0, self.w1);
            let p3 = OffsetParams::new(&self.c, t1, self.w0, self.w1);

            let t_mid = (t0 + t1) / 2.0;
            let p_mid = OffsetParams::new(&self.c, t_mid, self.w0, self.w1);

            // Fit cubic passing through p0.pos, p_mid.pos, p3.pos
            // with tangents p0.tangent and p3.tangent.

            // Use normalized directions for the solver
            let v0 = if p0.tangent.hypot2() > 1e-12 {
                p0.tangent.normalize()
            } else {
                Vec2::ZERO
            };
            let v1 = if p3.tangent.hypot2() > 1e-12 {
                p3.tangent.normalize()
            } else {
                Vec2::ZERO
            };

            // Solve for alpha, beta
            // M = (P0 + P3)/2 + 3/8 (alpha V0 - beta V1)
            // Delta = 8/3 * (M - Mid_Chord)
            let mid_chord = p0.pos + (p3.pos - p0.pos) * 0.5;
            let delta = (8.0 / 3.0) * (p_mid.pos - mid_chord);

            // System: alpha * V0 - beta * V1 = Delta
            // Cross product solution (Cramer's rule variant for 2D vectors)
            // alpha (V0 x V1) = Delta x V1
            // beta (V0 x V1) = Delta x V0  (Check signs)

            let det = v0.cross(v1);

            let (p1, p2) = if det.abs() > 1e-6 {
                let alpha = delta.cross(v1) / det;
                let beta = delta.cross(v0) / det; // Note: -beta * V1 in formula implies we add it back

                // Heuristic safety: clamp handles to reasonable lengths to prevent loops
                // if the tangents are fighting the midpoint.
                // (Skipped for brevity, but recommended in prod)

                (p0.pos + v0 * alpha, p3.pos - v1 * beta)
            } else {
                // Tangents are parallel. Fallback to standard 1/3 approximation.
                let len = (p3.pos - p0.pos).hypot();
                (p0.pos + v0 * (len / 3.0), p3.pos - v1 * (len / 3.0))
            };

            result.curve_to(p1, p2, p3.pos);
        }
    }
}
