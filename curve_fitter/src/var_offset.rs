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

/// Cubic Hermite width function over one segment, in the segment parameter t.
///
/// Every width profile reduces to this form: linear interpolation has end
/// slopes equal to `w1 - w0`, smoothstep has zero end slopes, and monotone
/// cubic carries the slopes of the global width curve. Slopes are in
/// dw/dt units of the segment parameter.
#[derive(Clone, Copy, Debug)]
pub struct SegmentWidth {
    pub w0: f64,
    pub w1: f64,
    pub m0: f64,
    pub m1: f64,
}

impl SegmentWidth {
    /// Plain linear width, the piecewise-linear profile
    pub fn linear(w0: f64, w1: f64) -> Self {
        let d = w1 - w0;
        Self {
            w0,
            w1,
            m0: d,
            m1: d,
        }
    }

    /// Width scaled by a factor (e.g. ±0.5 for the two offset sides)
    pub(crate) fn scaled(self, s: f64) -> Self {
        Self {
            w0: self.w0 * s,
            w1: self.w1 * s,
            m0: self.m0 * s,
            m1: self.m1 * s,
        }
    }

    /// Width at parameter t (cubic Hermite)
    pub fn eval(&self, t: f64) -> f64 {
        let t2 = t * t;
        let t3 = t2 * t;
        self.w0 * (2.0 * t3 - 3.0 * t2 + 1.0)
            + self.m0 * (t3 - 2.0 * t2 + t)
            + self.w1 * (-2.0 * t3 + 3.0 * t2)
            + self.m1 * (t3 - t2)
    }

    /// Width derivative dw/dt at parameter t
    pub fn deriv(&self, t: f64) -> f64 {
        let t2 = t * t;
        self.w0 * (6.0 * t2 - 6.0 * t)
            + self.m0 * (3.0 * t2 - 4.0 * t + 1.0)
            + self.w1 * (-6.0 * t2 + 6.0 * t)
            + self.m1 * (3.0 * t2 - 2.0 * t)
    }

    /// Whether this reduces to plain linear width (straight offset of a line)
    pub(crate) fn is_linear(&self) -> bool {
        let d = self.w1 - self.w0;
        (self.m0 - d).abs() < 1e-12 && (self.m1 - d).abs() < 1e-12
    }
}

struct OffsetParams {
    pos: Point,
    tangent: Vec2, // Un-normalized derivative vector of the offset curve
}

impl OffsetParams {
    fn new(c: &CubicBez, t: f64, width: &SegmentWidth) -> Self {
        let deriv = c.deriv().eval(t).to_vec2();
        let deriv2 = c.deriv().deriv().eval(t).to_vec2();

        let d = width.eval(t); // Width at t
        let d_prime = width.deriv(t); // Width derivative (w.r.t t)

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
/// The offset distance along the segment is given by `width` (a cubic
/// Hermite in t). The result has exactly [`NUM_SUBDIVISIONS`] cubic segments.
pub fn offset_cubic_variable(c: CubicBez, width: SegmentWidth, result: &mut BezPath) {
    result.truncate(0);

    let start = OffsetParams::new(&c, 0.0, &width);
    result.move_to(start.pos);

    for i in 0..NUM_SUBDIVISIONS {
        let t0 = i as f64 / NUM_SUBDIVISIONS as f64;
        let t1 = (i + 1) as f64 / NUM_SUBDIVISIONS as f64;
        fit_offset_segment(&c, t0, t1, &width, result);
    }
}

/// Fit one cubic to the offset curve over [t0, t1] and append it to `result`.
///
/// One-point shape control: the cubic interpolates the offset positions and
/// tangent directions at t0 and t1, with handle lengths solved so the curve
/// passes through the offset position at the interval midpoint.
fn fit_offset_segment(c: &CubicBez, t0: f64, t1: f64, width: &SegmentWidth, result: &mut BezPath) {
    let p0 = OffsetParams::new(c, t0, width);
    let p3 = OffsetParams::new(c, t1, width);

    let t_mid = (t0 + t1) / 2.0;
    let p_mid = OffsetParams::new(c, t_mid, width);

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
