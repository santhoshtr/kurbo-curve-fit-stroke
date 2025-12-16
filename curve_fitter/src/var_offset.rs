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
struct VariableCubicOffset {
    /// The cubic being offset.
    c: CubicBez,
    /// The derivative of `c`.
    q: QuadBez,
    /// Start width (at t=0)
    w0: f64,
    /// End width (at t=1)
    w1: f64,

    // Curvature factors (unscaled by width)
    // k0 + k1 t + k2 t^2 is the cross product of second and first derivatives.
    k0: f64,
    k1: f64,
    k2: f64,

    tolerance: f64,
}

// We never let cusp values have an absolute value smaller than this.
const CUSP_EPSILON: f64 = 1e-12;
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
pub fn offset_cubic_variable(
    c: CubicBez,
    w0: f64,
    w1: f64,

    // Optional locked tangents (normalized)
    tan0_override: Option<Vec2>,
    tan1_override: Option<Vec2>,
    tolerance: f64,
    result: &mut BezPath,
) {
    // Regularization tuning
    const DIM_TUNE: f64 = 0.25;

    // We regularize using the maximum width to be safe against cusps
    let max_w = if w0.abs() > w1.abs() { w0 } else { w1 };
    let c_regularized = c.regularize_cusp(tolerance * DIM_TUNE);

    let co = VariableCubicOffset::new(c_regularized, w0, w1, tolerance);

    let (tan0, tan1) = tangents(&PathSeg::Cubic(c));
    // Use override if provided, otherwise derive from source
    let utan0 = tan0_override.unwrap_or_else(|| tan0.normalize());
    let utan1 = tan1_override.unwrap_or_else(|| tan1.normalize());

    let cusp0 = co.endpoint_cusp(co.q.p0, co.k0, w0);
    let cusp1 = co.endpoint_cusp(co.q.p2, co.k0 + co.k1 + co.k2, w1);

    // Start point uses w0 explicitly
    result.move_to(c.p0 + w0 * utan0.turn_90());

    let rec = OffsetRec::new(0., 1., utan0, utan1, cusp0, cusp1, 0);
    co.offset_rec(&rec, result);
}

impl VariableCubicOffset {
    fn new(c: CubicBez, w0: f64, w1: f64, tolerance: f64) -> Self {
        let q = c.deriv();
        // Calculate curvature cross-product factors (unscaled by width)
        // The factor 2.0 comes from the second derivative of cubic
        let p1xp0 = q.p1.to_vec2().cross(q.p0.to_vec2());
        let p2xp0 = q.p2.to_vec2().cross(q.p0.to_vec2());
        let p2xp1 = q.p2.to_vec2().cross(q.p1.to_vec2());

        VariableCubicOffset {
            c,
            q,
            w0,
            w1,
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

    fn cusp_sign(&self, t: f64) -> f64 {
        let ds2 = self.q.eval(t).to_vec2().hypot2();
        let curvature_factor = (self.k2 * t + self.k1) * t + self.k0;
        let w = self.width(t);
        // curvature * width + 1
        (curvature_factor * w) / (ds2 * ds2.sqrt()) + 1.0
    }

    fn endpoint_cusp(&self, tan: Point, k_val: f64, w_val: f64) -> f64 {
        const TAN_DIST_EPSILON: f64 = 1e-12;
        let tan_dist = tan.to_vec2().hypot().max(TAN_DIST_EPSILON);
        let rsqrt = 1.0 / tan_dist;
        // k_val is the curvature numerator, w_val is the width
        (k_val * w_val) * (rsqrt * rsqrt * rsqrt) + 1.0
    }

    fn offset_rec(&self, rec: &OffsetRec, result: &mut BezPath) {
        // Cusp detection logic remains mostly the same, but using dynamic width
        if rec.cusp0 * rec.cusp1 < 0.0 {
            let a = rec.t0;
            let b = rec.t1;
            let s = rec.cusp1.signum();
            let f = |t| s * self.cusp_sign(t);
            let k1 = 0.2 / (b - a);
            const ITP_EPS: f64 = 1e-12;
            let t = solve_itp(f, a, b, ITP_EPS, 1, k1, s * rec.cusp0, s * rec.cusp1);

            let utan_t = self.q.eval(t).to_vec2().normalize();
            let cusp_t_minus = CUSP_EPSILON.copysign(rec.cusp0);
            let cusp_t_plus = CUSP_EPSILON.copysign(rec.cusp1);
            self.subdivide(rec, result, t, utan_t, cusp_t_minus, cusp_t_plus);
            return;
        }

        let (mut a, mut b) = self.draw_arc(rec);
        let dt = (rec.t1 - rec.t0) * (1.0 / (N_LSE + 1) as f64);
        let mut ts = core::array::from_fn(|i| rec.t0 + (i + 1) as f64 * dt);

        let mut c_approx = self.apply(rec, a, b);
        let err_init = self.eval_err(rec, c_approx, &mut ts);
        let mut err = err_init;

        const N_REFINE: usize = 2;
        for _ in 0..N_REFINE {
            if err.err_squared <= self.tolerance * self.tolerance {
                break;
            }
            let (a2, b2) = self.refine_least_squares(rec, a, b, &err);
            let c_approx2 = self.apply(rec, a2, b2);
            let err2 = self.eval_err(rec, c_approx2, &mut ts);
            if err2.err_squared >= err.err_squared {
                break;
            }
            err = err2;
            (a, b) = (a2, b2);
            c_approx = c_approx2;
        }

        if rec.depth < MAX_DEPTH && err.err_squared > self.tolerance * self.tolerance {
            let SubdivisionPoint { t, utan } = self.find_subdivision_point(rec);
            let cusp = self.cusp_sign(t);
            self.subdivide(rec, result, t, utan, cusp, cusp);
        } else {
            result.curve_to(c_approx.p1, c_approx.p2, c_approx.p3);
        }
    }

    fn subdivide(
        &self,
        rec: &OffsetRec,
        result: &mut BezPath,
        t: f64,
        utan_t: Vec2,
        cusp_t_minus: f64,
        cusp_t_plus: f64,
    ) {
        let rec0 = OffsetRec::new(
            rec.t0,
            t,
            rec.utan0,
            utan_t,
            rec.cusp0,
            cusp_t_minus,
            rec.depth + 1,
        );
        self.offset_rec(&rec0, result);
        let rec1 = OffsetRec::new(
            t,
            rec.t1,
            utan_t,
            rec.utan1,
            cusp_t_plus,
            rec.cusp1,
            rec.depth + 1,
        );
        self.offset_rec(&rec1, result);
    }

    // Apply uses w(t0) and w(t1) specifically
    fn apply(&self, rec: &OffsetRec, a: f64, b: f64) -> CubicBez {
        let s = (1. / 3.) * (rec.t1 - rec.t0);

        let w_start = self.width(rec.t0);
        let w_end = self.width(rec.t1);

        let p0 = self.c.eval(rec.t0) + w_start * rec.utan0.turn_90();
        // l0 is the handle length.
        // Note: The logic `a * d` in original code scales the tangent modification by width.
        // We use the local width w_start for this scaling.
        let l0 = s * self.q.eval(rec.t0).to_vec2().length() + a * w_start;

        let mut p1 = p0;
        if l0 * rec.cusp0 > 0.0 {
            p1 += l0 * rec.utan0;
        }

        let p3 = self.c.eval(rec.t1) + w_end * rec.utan1.turn_90();
        let mut p2 = p3;
        // Similarly here using w_end
        let l1 = s * self.q.eval(rec.t1).to_vec2().length() - b * w_end;

        if l1 * rec.cusp1 > 0.0 {
            p2 -= l1 * rec.utan1;
        }
        CubicBez::new(p0, p1, p2, p3)
    }

    fn draw_arc(&self, rec: &OffsetRec) -> (f64, f64) {
        let th = rec.utan1.cross(rec.utan0).atan2(rec.utan1.dot(rec.utan0));
        let a = (2. / 3.) / (1.0 + (0.5 * th).cos()) * 2.0 * (0.5 * th).sin();
        let b = -a;
        (a, b)
    }

    fn eval_err(&self, rec: &OffsetRec, c_approx: CubicBez, ts: &mut [f64; N_LSE]) -> ErrEval {
        let qa = c_approx.deriv();
        let mut err_squared = 0.0;
        let mut unorms = [Vec2::ZERO; N_LSE];
        let mut err_vecs = [Vec2::ZERO; N_LSE];

        for i in 0..N_LSE {
            let ta = (i + 1) as f64 * (1.0 / (N_LSE + 1) as f64);
            let mut t = ts[i];
            let p = self.c.eval(t);
            let pa = c_approx.eval(ta);
            let tana = qa.eval(ta).to_vec2();
            t += tana.dot(pa - p) / tana.dot(self.q.eval(t).to_vec2());
            t = t.max(rec.t0).min(rec.t1);
            ts[i] = t;

            let cusp = rec.cusp0.signum();
            let unorm = cusp * tana.normalize().turn_90();
            unorms[i] = unorm;

            // Critical change: Compare against source + variable width(t)
            let w_t = self.width(t);
            let p_new = self.c.eval(t) + w_t * unorm;

            let err_vec = pa - p_new;
            err_vecs[i] = err_vec;
            let mut dist_err_squared = err_vec.length_squared();
            if !dist_err_squared.is_finite() {
                dist_err_squared = 1e12;
            }
            err_squared = dist_err_squared.max(err_squared);
        }
        ErrEval {
            err_squared,
            unorms,
            err_vecs,
        }
    }

    fn refine_least_squares(&self, rec: &OffsetRec, a: f64, b: f64, err: &ErrEval) -> (f64, f64) {
        let mut aa = 0.0;
        let mut ab = 0.0;
        let mut ac = 0.0;
        let mut bb = 0.0;
        let mut bc = 0.0;

        for i in 0..N_LSE {
            let n = err.unorms[i];
            let err_vec = err.err_vecs[i];
            let c_n = err_vec.dot(n);
            let c_t = err_vec.cross(n);
            let a_n = A_WEIGHTS[i] * rec.utan0.dot(n);
            let a_t = A_WEIGHTS[i] * rec.utan0.cross(n);
            let b_n = B_WEIGHTS[i] * rec.utan1.dot(n);
            let b_t = B_WEIGHTS[i] * rec.utan1.cross(n);
            aa += a_n * a_n + BLEND * (a_t * a_t);
            ab += a_n * b_n + BLEND * a_t * b_t;
            ac += a_n * c_n + BLEND * a_t * c_t;
            bb += b_n * b_n + BLEND * (b_t * b_t);
            bc += b_n * c_n + BLEND * b_t * c_t;
        }

        // Critical change: Inverse determinant scaling.
        // Original used `self.d`. We use average width of the segment for stability.
        let w_avg = (self.width(rec.t0) + self.width(rec.t1)) * 0.5;
        // Avoid division by zero if width is 0
        let scale = if w_avg.abs() < 1e-9 { 1.0 } else { w_avg };

        let idet = 1.0 / (scale * (aa * bb - ab * ab));
        let delta_a = idet * (ac * bb - ab * bc);
        let delta_b = idet * (aa * bc - ac * ab);
        (a - delta_a, b - delta_b)
    }

    fn find_subdivision_point(&self, rec: &OffsetRec) -> SubdivisionPoint {
        // This logic remains purely geometric based on the source curve
        // so it doesn't strictly need modification for variable width,
        // as we want to subdivide where the SOURCE curve is weird.
        let t = 0.5 * (rec.t0 + rec.t1);
        let q_t = self.q.eval(t).to_vec2();
        let x0 = rec.utan0.cross(q_t).abs();
        let x1 = rec.utan1.cross(q_t).abs();
        const SUBDIVIDE_THRESH: f64 = 0.1;
        if x0 > SUBDIVIDE_THRESH * x1 && x1 > SUBDIVIDE_THRESH * x0 {
            let utan = q_t.normalize();
            return SubdivisionPoint { t, utan };
        }

        let chord = self.c.eval(rec.t1) - self.c.eval(rec.t0);
        if chord.cross(rec.utan0) * chord.cross(rec.utan1) < 0.0 {
            let tan = rec.utan0 + rec.utan1;
            if let Some(subdivision) =
                self.subdivide_for_tangent(rec.utan0, rec.t0, rec.t1, tan, false)
            {
                return subdivision;
            }
        }

        let mut tangents: ArrayVec<Vec2, 4> = ArrayVec::new();
        let mut ts: ArrayVec<f64, 4> = ArrayVec::new();
        tangents.push(rec.utan0);
        ts.push(rec.t0);
        for t in self.c.inflections() {
            if t > rec.t0 && t < rec.t1 {
                tangents.push(self.q.eval(t).to_vec2());
                ts.push(t);
            }
        }
        tangents.push(rec.utan1);
        ts.push(rec.t1);
        let mut arc_angles: ArrayVec<f64, 3> = ArrayVec::new();
        let mut sum = 0.0;
        for i in 0..tangents.len() - 1 {
            let tan0 = tangents[i];
            let tan1 = tangents[i + 1];
            let th = tan0.cross(tan1).atan2(tan0.dot(tan1));
            sum += th.abs();
            arc_angles.push(th);
        }
        let mut target = sum * 0.5;
        let mut i = 0;
        while arc_angles[i].abs() < target {
            target -= arc_angles[i].abs();
            i += 1;
        }
        let rotation = Vec2::from_angle(target.copysign(arc_angles[i]));
        let base = tangents[i];
        let tan = base.rotate_scale(rotation);
        let utan0 = if i == 0 { rec.utan0 } else { base.normalize() };
        self.subdivide_for_tangent(utan0, ts[i], ts[i + 1], tan, true)
            .unwrap()
    }

    fn subdivide_for_tangent(
        &self,
        utan0: Vec2,
        t0: f64,
        t1: f64,
        tan: Vec2,
        force: bool,
    ) -> Option<SubdivisionPoint> {
        let mut t = 0.0;
        let mut n_soln = 0;
        let z0 = tan.cross(self.q.p0.to_vec2());
        let z1 = tan.cross(self.q.p1.to_vec2());
        let z2 = tan.cross(self.q.p2.to_vec2());
        let c0 = z0;
        let c1 = 2.0 * (z1 - z0);
        let c2 = (z2 - z1) - (z1 - z0);
        for root in solve_quadratic(c0, c1, c2) {
            if root >= t0 && root <= t1 {
                t = root;
                n_soln += 1;
            }
        }
        if n_soln != 1 {
            if !force {
                return None;
            }
            if self.q.eval(t0).to_vec2().length_squared()
                > self.q.eval(t1).to_vec2().length_squared()
            {
                t = t1;
            } else {
                t = t0;
            }
        }
        let q = self.q.eval(t).to_vec2();
        const UTAN_EPSILON: f64 = 1e-12;
        let utan = if n_soln == 1 && q.length_squared() >= UTAN_EPSILON {
            q.normalize()
        } else if tan.length_squared() >= UTAN_EPSILON {
            tan.normalize()
        } else {
            utan0.turn_90()
        };
        Some(SubdivisionPoint { t, utan })
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
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_offset_cubic_variable_simple_line() {
        // Test a simple horizontal line
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 0.0),
            Point::new(66.6, 0.0),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, 5.0, 5.0, None, None, 0.1, &mut result);

        println!("SVG path: {}", result.to_svg());
        // Should produce a path with at least one element
        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_offset_cubic_variable_varying_width() {
        // Test with varying width from 5 to 15
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 0.0),
            Point::new(66.6, 0.0),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, 5.0, 10.0, None, None, 0.1, &mut result);

        println!("SVG path: {}", result.to_svg());
        // Should produce a path
        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_offset_cubic_variable_curved_path() {
        // Test with an actual curve
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(0.0, 50.0),
            Point::new(100.0, 50.0),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, 10.0, 12.0, None, None, 0.1, &mut result);
        println!("{}", result.to_svg());
        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_offset_cubic_variable_negative_width() {
        // Test with negative width (offset in opposite direction)
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 0.0),
            Point::new(66.6, 0.0),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, -5.0, -5.0, None, None, 0.1, &mut result);

        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_offset_cubic_variable_zero_width() {
        // Test with zero width at one end
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 0.0),
            Point::new(66.6, 0.0),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, 0.0, 10.0, None, None, 0.1, &mut result);

        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_variable_cubic_offset_width_interpolation() {
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 0.0),
            Point::new(66.6, 0.0),
            Point::new(100.0, 0.0),
        );

        let offset = VariableCubicOffset::new(c, 10.0, 20.0, 0.1);

        // Test width interpolation
        assert!((offset.width(0.0) - 10.0).abs() < 1e-10);
        assert!((offset.width(1.0) - 20.0).abs() < 1e-10);
        assert!((offset.width(0.5) - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_variable_cubic_offset_cusp_sign() {
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(0.0, 50.0),
            Point::new(100.0, 50.0),
            Point::new(100.0, 0.0),
        );

        let offset = VariableCubicOffset::new(c, 10.0, 10.0, 0.1);

        // Cusp sign should be finite
        let cusp = offset.cusp_sign(0.5);
        assert!(cusp.is_finite());
    }

    #[test]
    fn test_variable_cubic_offset_endpoint_cusp() {
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 0.0),
            Point::new(66.6, 0.0),
            Point::new(100.0, 0.0),
        );

        let offset = VariableCubicOffset::new(c, 10.0, 10.0, 0.1);

        let cusp0 = offset.endpoint_cusp(offset.q.p0, offset.k0, 10.0);
        let cusp1 = offset.endpoint_cusp(offset.q.p2, offset.k0 + offset.k1 + offset.k2, 10.0);

        assert!(cusp0.is_finite());
        assert!(cusp1.is_finite());
    }

    #[test]
    fn test_offset_cubic_variable_s_curve() {
        // Test S-shaped curve
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(50.0, -100.0),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, 8.0, 12.0, None, None, 0.1, &mut result);

        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_offset_cubic_variable_tolerance() {
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(0.0, 50.0),
            Point::new(100.0, 50.0),
            Point::new(100.0, 0.0),
        );

        // Test with different tolerances
        let mut result_low_tol = BezPath::new();
        offset_cubic_variable(c, 10.0, 10.0, None, None, 0.01, &mut result_low_tol);

        let mut result_high_tol = BezPath::new();
        offset_cubic_variable(c, 10.0, 10.0, None, None, 1.0, &mut result_high_tol);

        // Both should produce valid paths
        assert!(result_low_tol.elements().len() > 0);
        assert!(result_high_tol.elements().len() > 0);

        // Lower tolerance might produce more elements (more precise)
        // but this is not guaranteed, so we just check they're both valid
    }

    #[test]
    fn test_offset_rec_new() {
        let rec = OffsetRec::new(
            0.0,
            1.0,
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 0.0),
            1.0,
            1.0,
            0,
        );

        assert_eq!(rec.t0, 0.0);
        assert_eq!(rec.t1, 1.0);
        assert_eq!(rec.depth, 0);
    }

    #[test]
    fn test_variable_cubic_offset_large_width_variation() {
        // Test with large width variation
        let c = CubicBez::new(
            Point::new(0.0, 0.0),
            Point::new(33.3, 33.3),
            Point::new(66.6, 33.3),
            Point::new(100.0, 0.0),
        );

        let mut result = BezPath::new();
        offset_cubic_variable(c, 5.0, 50.0, None, None, 0.1, &mut result);

        assert!(result.elements().len() > 0);
    }
}
