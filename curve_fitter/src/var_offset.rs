// Copyright 2022 the Kurbo Authors (Modified for Variable Width)
// SPDX-License-Identifier: Apache-2.0 OR MIT
use kurbo::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, Point, Vec2};

/// Number of cubic segments generated per source segment.
///
/// The count is fixed (rather than adaptive to an error tolerance) so that
/// outlines stroked from the same skeleton with different widths always have
/// the same number of points, which keeps them interpolatable for variable
/// fonts.
const NUM_SUBDIVISIONS: usize = 2;

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

        // Offset derivative: x'_{off} = (1 - k*d)x' + n*d'
        // (differentiating o(t) = c(t) + d(t)*n(t); dn/dt = -k*c' with the CCW normal)
        let term_a = (1.0 - curvature * d) * deriv;
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
/// Width is interpolated linearly with t between w0 and w1. The result has
/// exactly [`NUM_SUBDIVISIONS`] cubic segments.
pub fn offset_cubic_variable(c: CubicBez, w0: f64, w1: f64, result: &mut BezPath) {
    result.truncate(0);

    let start = OffsetParams::new(&c, 0.0, w0, w1);
    result.move_to(start.pos);

    for i in 0..NUM_SUBDIVISIONS {
        let t0 = i as f64 / NUM_SUBDIVISIONS as f64;
        let t1 = (i + 1) as f64 / NUM_SUBDIVISIONS as f64;
        fit_offset_segment(&c, t0, t1, w0, w1, result);
    }
}

/// Fit one cubic to the offset curve over [t0, t1] and append it to `result`.
///
/// One-point shape control: the cubic interpolates the offset positions and
/// tangent directions at t0 and t1, with handle lengths solved so the curve
/// passes through the offset position at the interval midpoint.
fn fit_offset_segment(c: &CubicBez, t0: f64, t1: f64, w0: f64, w1: f64, result: &mut BezPath) {
    let p0 = OffsetParams::new(c, t0, w0, w1);
    let p3 = OffsetParams::new(c, t1, w0, w1);

    let t_mid = (t0 + t1) / 2.0;
    let p_mid = OffsetParams::new(c, t_mid, w0, w1);

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
    // beta (V0 x V1) = Delta x V0

    let det = v0.cross(v1);

    let (p1, p2) = if det.abs() > 1e-6 {
        let alpha = delta.cross(v1) / det;
        let beta = delta.cross(v0) / det;

        // Note: alpha/beta are not clamped; extreme tangent/midpoint
        // combinations can produce handles that make the segment self-loop.
        (p0.pos + v0 * alpha, p3.pos - v1 * beta)
    } else {
        // Tangents are parallel. Fallback to standard 1/3 approximation.
        let len = (p3.pos - p0.pos).hypot();
        (p0.pos + v0 * (len / 3.0), p3.pos - v1 * (len / 3.0))
    };

    result.curve_to(p1, p2, p3.pos);
}
