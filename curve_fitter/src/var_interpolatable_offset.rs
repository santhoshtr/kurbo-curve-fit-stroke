// Copyright 2022 the Kurbo Authors (Modified for Variable Width)
// SPDX-License-Identifier: Apache-2.0 OR MIT

use arrayvec::ArrayVec;
use kurbo::{
    BezPath, CubicBez, CuspType, ParamCurve, ParamCurveDeriv, ParamCurveNearest, PathSeg, Point,
    QuadBez, Vec2,
    common::{solve_itp, solve_quadratic},
};

use crate::tangents;

/// State used for computing a variable-width offset curve of a single cubic.
struct VariableInterpolatableCubicOffset {
    /// The cubic being offset.
    c: CubicBez,
    /// The derivative of `c`.
    q: QuadBez,
    /// Start width (at t=0)
    w0: f64,
    /// End width (at t=1)
    w1: f64,
    sub_divisions: usize,
    // Curvature factors (unscaled by width)
    // k0 + k1 t + k2 t^2 is the cross product of second and first derivatives.
    k0: f64,
    k1: f64,
    k2: f64,

    tolerance: f64,
}

// We never let cusp values have an absolute value smaller than this.
const N_LSE: usize = 8;
const BLEND: f64 = 1e-3;
const MAX_DEPTH: usize = 8;

/// State local to a subdivision
struct OffsetRec {
    t0: f64,
    t1: f64,
    // unit tangent at t0
    utan0: Vec2,
    // unit tangent at t1
    utan1: Vec2,
    cusp0: f64,
    cusp1: f64,
    /// Recursion depth
    depth: usize,
}

/// Result of error evaluation
struct ErrEval {
    err_squared: f64,
    unorms: [Vec2; N_LSE],
    err_vecs: [Vec2; N_LSE],
}

/// Result of subdivision
struct SubdivisionPoint {
    t: f64,
    utan: Vec2,
}

/// Compute an approximate variable-width offset curve.
///
/// * `w0`: The offset distance at t=0.
/// * `w1`: The offset distance at t=1.
///
/// Width is interpolated linearly with t between w0 and w1.
pub fn offset_cubic_interpolatable_variable(
    c: CubicBez,
    w0: f64,
    w1: f64,

    // Optional locked tangents (normalized)
    tan0_override: Option<Vec2>,
    tan1_override: Option<Vec2>,
    subdivisions: usize,
    tolerance: f64,
    result: &mut BezPath,
) {
    result.truncate(0);

    // We do NOT regularize or split at cusps adaptively.
    // That would break point compatibility.
    // If the source has a cusp, the user should have split the source beforehand.

    let offset = VariableInterpolatableCubicOffset::new(c, w0, w1, subdivisions, tolerance);

    // Calculate initial MoveTo
    let t0 = 0.0;
    let q0 = c.deriv().eval(t0).to_vec2();
    let n0 = if q0.hypot() > 1e-9 {
        q0.normalize().turn_90()
    } else {
        Vec2::ZERO
    }; // Simplified

    result.move_to(c.p0 + w0 * n0);

    offset.recurse(0.0, 1.0, 0, subdivisions, result);
}

impl VariableInterpolatableCubicOffset {
    fn new(c: CubicBez, w0: f64, w1: f64, sub_divisions: usize, tolerance: f64) -> Self {
        let q = c.deriv();
        // Calculate curvature cross-product factors (unscaled by width)
        // The factor 2.0 comes from the second derivative of cubic
        let p1xp0 = q.p1.to_vec2().cross(q.p0.to_vec2());
        let p2xp0 = q.p2.to_vec2().cross(q.p0.to_vec2());
        let p2xp1 = q.p2.to_vec2().cross(q.p1.to_vec2());

        VariableInterpolatableCubicOffset {
            c,
            q,
            w0,
            w1,
            sub_divisions,
            k0: 2.0 * p1xp0,
            k1: 2.0 * (p2xp0 - 2.0 * p1xp0),
            k2: 2.0 * (p2xp1 - p2xp0 + p1xp0),
            tolerance,
        }
    }

    /// Get the width at a specific t value.
    #[inline]
    fn width(&self, t: f64) -> f64 {
        self.w0 + (self.w1 - self.w0) * t
    }

    /// Recursively split until we hit target depth, then generate a segment.
    fn recurse(&self, t0: f64, t1: f64, depth: usize, target_depth: usize, result: &mut BezPath) {
        if depth < target_depth {
            // STRICT TOPOLOGY: Always split exactly in half.
            let mid = (t0 + t1) / 2.0;
            self.recurse(t0, mid, depth + 1, target_depth, result);
            self.recurse(mid, t1, depth + 1, target_depth, result);
        } else {
            // LEAF NODE: Generate the curve
            self.emit_leaf(t0, t1, result);
        }
    }

    fn emit_leaf(&self, t0: f64, t1: f64, result: &mut BezPath) {
        // 1. Calculate Geometry at endpoints
        let p0_src = self.c.eval(t0);
        let p3_src = self.c.eval(t1);

        let q0 = self.q.eval(t0).to_vec2();
        let q1 = self.q.eval(t1).to_vec2();

        // Handle zero derivatives (rare in well-formed fonts, but safe to handle)
        let utan0 = if q0.hypot() > 1e-9 {
            q0.normalize()
        } else {
            Vec2::new(1.0, 0.0)
        };
        let utan1 = if q1.hypot() > 1e-9 {
            q1.normalize()
        } else {
            Vec2::new(1.0, 0.0)
        };

        let n0 = utan0.turn_90();
        let n1 = utan1.turn_90();

        // 2. Calculate Exact Offset Points (On-Curve)
        let w_start = self.width(t0);
        let w_end = self.width(t1);

        let p0 = p0_src + w_start * n0;
        let p3 = p3_src + w_end * n1;

        // 3. Approximate Control Points
        // We use Kurbo's "Arc Drawing" heuristic (Hoschek-like)
        // to find the optimal handle length 'a' and 'b'.

        // s = Scaling factor from t-space to Euclidean space
        let s = (t1 - t0) / 3.0;

        // "Geometric" tangents (tangent of the source curve)
        // For a generic variable offset, the true tangent rotates slightly if width changes rapidly.
        // However, for interpolation stability, locking to the geometric tangent is safer.

        // Calculate the "Arc" parameters
        let th = utan1.cross(utan0).atan2(utan1.dot(utan0));
        let mut k_a = (2. / 3.) / (1.0 + (0.5 * th).cos()) * 2.0 * (0.5 * th).sin();
        // If angle is near zero, k_a -> 0/0, limit is 0.
        if !k_a.is_finite() {
            k_a = 0.0;
        }

        let k_b = -k_a; // Symmetric approximation

        // The Kurbo `apply` logic adapted for fixed widths:
        // Handle Length = (Geometric deriv length) * (1 + width_correction?)
        // To ensure 100% stability, we use a simpler Tiller-Hanson-ish approach
        // refined by the arc drawing factor.

        let len0 = self.q.eval(t0).to_vec2().length();
        let len1 = self.q.eval(t1).to_vec2().length();

        // l0/l1 are the handle lengths.
        // Formula: s * derivative_len + curvature_adjust
        // We simply apply the Arc factor (k_a) scaled by the width to "bend" the curve
        // parallel to the source.

        let l0 = s * len0 + k_a * w_start;
        let l1 = s * len1 - k_b * w_end;

        let p1 = p0 + l0 * utan0;
        let p2 = p3 - l1 * utan1;

        result.curve_to(p1, p2, p3);
    }
}

impl OffsetRec {
    fn new(
        t0: f64,
        t1: f64,
        utan0: Vec2,
        utan1: Vec2,
        cusp0: f64,
        cusp1: f64,
        depth: usize,
    ) -> Self {
        OffsetRec {
            t0,
            t1,
            utan0,
            utan1,
            cusp0,
            cusp1,
            depth,
        }
    }
}

// Weights borrowed from original Kurbo implementation
const fn mk_a_weights(rev: bool) -> [f64; N_LSE] {
    let mut result = [0.0; N_LSE];
    let mut i = 0;
    while i < N_LSE {
        let t = (i + 1) as f64 / (N_LSE + 1) as f64;
        let mt = 1. - t;
        let ix = if rev { N_LSE - 1 - i } else { i };
        result[ix] = 3.0 * mt * t * mt;
        i += 1;
    }
    result
}

const A_WEIGHTS: [f64; N_LSE] = mk_a_weights(false);
const B_WEIGHTS: [f64; N_LSE] = mk_a_weights(true);

// Following code is already present in kurbo. Copying here to make it public
trait NewTrait {
    fn detect_cusp(&self, dimension: f64) -> Option<CuspType>;
    /// Preprocess a cubic Bézier to ease numerical robustness.
    ///
    /// If the cubic Bézier segment has zero or near-zero derivatives as an interior
    /// cusp, perturb the control points to make curvature finite, avoiding
    /// numerical robustness problems in offset and stroke.
    fn regularize_cusp(&self, dimension: f64) -> CubicBez;
}

impl NewTrait for CubicBez {
    /// Preprocess a cubic Bézier to ease numerical robustness.
    ///
    /// If the cubic Bézier segment has zero or near-zero derivatives as an interior
    /// cusp, perturb the control points to make curvature finite, avoiding
    /// numerical robustness problems in offset and stroke.
    fn regularize_cusp(&self, dimension: f64) -> CubicBez {
        let mut c = *self;
        // First step: if control point is too near the endpoint, nudge it away
        // along the tangent.
        if let Some(cusp_type) = self.detect_cusp(dimension) {
            let d01 = c.p1 - c.p0;
            let d01h = d01.hypot();
            let d23 = c.p3 - c.p2;
            let d23h = d23.hypot();
            match cusp_type {
                CuspType::Loop => {
                    c.p1 += (dimension / d01h) * d01;
                    c.p2 -= (dimension / d23h) * d23;
                }
                CuspType::DoubleInflection => {
                    // Avoid making control distance smaller than dimension
                    if d01h > 2.0 * dimension {
                        c.p1 -= (dimension / d01h) * d01;
                    }
                    if d23h > 2.0 * dimension {
                        c.p2 += (dimension / d23h) * d23;
                    }
                }
            }
        }
        c
    }
    /// Detect whether there is a cusp.
    ///
    /// Return a cusp classification if there is a cusp with curvature greater than
    /// the reciprocal of the given dimension.
    fn detect_cusp(&self, dimension: f64) -> Option<CuspType> {
        let d01 = self.p1 - self.p0;
        let d02 = self.p2 - self.p0;
        let d03 = self.p3 - self.p0;
        let d12 = self.p2 - self.p1;
        let d23 = self.p3 - self.p2;
        let det_012 = d01.cross(d02);
        let det_123 = d12.cross(d23);
        let det_013 = d01.cross(d03);
        let det_023 = d02.cross(d03);
        if det_012 * det_123 > 0.0 && det_012 * det_013 < 0.0 && det_012 * det_023 < 0.0 {
            let q = self.deriv();
            // accuracy isn't used for quadratic nearest
            let nearest = q.nearest(Point::ORIGIN, 1e-9);
            // detect whether curvature at minimum derivative exceeds 1/dimension,
            // without division.
            let d = q.eval(nearest.t);
            let d2 = q.deriv().eval(nearest.t);
            let cross = d.to_vec2().cross(d2.to_vec2());
            if nearest.distance_sq.powi(3) <= (cross * dimension).powi(2) {
                let a = 3. * det_012 + det_023 - 2. * det_013;
                let b = -3. * det_012 + det_013;
                let c = det_012;
                let d = b * b - 4. * a * c;
                if d > 0.0 {
                    return Some(CuspType::DoubleInflection);
                } else {
                    return Some(CuspType::Loop);
                }
            }
        }
        None
    }
}
