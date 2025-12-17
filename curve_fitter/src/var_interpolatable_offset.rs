// Copyright 2022 the Kurbo Authors (Modified for Variable Width)
// SPDX-License-Identifier: Apache-2.0 OR MIT
use kurbo::{BezPath, CubicBez, ParamCurve, ParamCurveDeriv, QuadBez, Vec2};
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
    // Curvature factors (unscaled by width)
    // k0 + k1 t + k2 t^2 is the cross product of second and first derivatives.
    k0: f64,
    k1: f64,
    k2: f64,

    tolerance: f64,
}

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

#[inline]
fn angle_between(v1: Vec2, v2: Vec2) -> f64 {
    let dot = v1.dot(v2);
    let cross = v1.cross(v2);
    cross.atan2(dot)
}

/// Calculate subdivision depth (0, 1, or 2) based on the source curve's curvature.
///
/// This depends ONLY on the source geometry, ensuring that different stroke weights
/// (which share the same source path) will produce compatible topologies.
fn calculate_dynamic_depth(c: CubicBez) -> usize {
    // 1. Get delta vectors
    let d0 = c.p1 - c.p0;
    let d1 = c.p2 - c.p1;
    let d2 = c.p3 - c.p2;

    // 2. Handle degenerate cases (coincidental points)
    // If control points overlap, fallback to checking the next available vector
    // or return a safe default (1).
    let v0 = if d0.hypot2() > 1e-9 { d0 } else { d1 };
    let v1 = if d1.hypot2() > 1e-9 {
        d1
    } else {
        if d0.hypot2() > 1e-9 { d0 } else { d2 }
    };
    let v2 = if d2.hypot2() > 1e-9 { d2 } else { v1 };

    // If the curve is essentially a point or a line, no subdivision needed.
    if v0.hypot2() < 1e-9 || v2.hypot2() < 1e-9 {
        return 0;
    }

    // 3. Calculate angles
    // Angle between v0 and v1
    let angle1 = angle_between(v0, v1).abs();
    // Angle between v1 and v2
    let angle2 = angle_between(v1, v2).abs();

    // 4. Total geometric turn
    let total_turn = angle1 + angle2;

    // 5. Thresholds (in Radians)
    // 60 degrees ~= 1.05 rad
    // 120 degrees ~= 2.1 rad
    if total_turn < 1.05 {
        0
    } else if total_turn < 2.1 {
        1
    } else {
        2
    }
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

    tolerance: f64,
    result: &mut BezPath,
) {
    result.truncate(0);

    // We do NOT regularize or split at cusps adaptively.
    // That would break point compatibility.
    // If the source has a cusp, the user should have split the source beforehand.
    let offset = VariableInterpolatableCubicOffset::new(
        c, w0, w1, // Store optional overrides
        tolerance,
    );

    // Calculate initial MoveTo
    let t0 = 0.0;
    let q0 = c.deriv().eval(t0).to_vec2();
    let n0 = if q0.hypot() > 1e-9 {
        q0.normalize().turn_90()
    } else {
        Vec2::ZERO
    }; // Simplified

    result.move_to(c.p0 + w0 * n0);

    offset.recurse(0.0, 1.0, 0, result);
}

impl VariableInterpolatableCubicOffset {
    fn new(c: CubicBez, w0: f64, w1: f64, tolerance: f64) -> Self {
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

    /// Calculate subdivision depth (0, 1, or 2) based on the source curve's curvature.
    ///
    /// This depends ONLY on the source geometry, ensuring that different stroke weights
    /// (which share the same source path) will produce compatible topologies.
    fn get_depth(&self) -> usize {
        calculate_dynamic_depth(self.c)
    }

    /// Recursively split until we hit target depth, then generate a segment.
    fn recurse(&self, t0: f64, t1: f64, depth: usize, result: &mut BezPath) {
        let target_depth = self.get_depth();
        if depth < target_depth {
            // STRICT TOPOLOGY: Always split exactly in half.
            let mid = (t0 + t1) / 2.0;
            self.recurse(t0, mid, depth + 1, result);
            self.recurse(mid, t1, depth + 1, result);
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

#[cfg(test)]
mod tests {
    use super::*;
    use kurbo::CubicBez;

    #[test]
    fn test_depth_straight_line() {
        let c = CubicBez::new((0., 0.), (10., 0.), (20., 0.), (30., 0.));
        assert_eq!(calculate_dynamic_depth(c), 0);
    }

    #[test]
    fn test_depth_gentle_curve() {
        // Turn is very small (< 10 degrees)
        let c = CubicBez::new((0., 0.), (10., 1.), (20., 1.), (30., 0.));
        assert_eq!(calculate_dynamic_depth(c), 0);
    }

    #[test]
    fn test_depth_sharp_corner() {
        // ~90 degree turn
        let c = CubicBez::new((0., 0.), (10., 0.), (10., 10.), (10., 20.));
        assert_eq!(calculate_dynamic_depth(c), 1);
    }

    #[test]
    fn test_depth_aggressive_s_curve() {
        // This was the failing test.
        // The control polygon zig-zags significantly (~216 degrees total).
        // It SHOULD be Depth 2.
        let c = CubicBez::new((0.0, 0.0), (10.0, 10.0), (20.0, -10.0), (30.0, 0.0));
        assert_eq!(calculate_dynamic_depth(c), 2);
    }

    #[test]
    fn test_depth_gentle_s_curve() {
        // A much gentler S-curve.
        // P0->P1: Rise 3 over 10 (~16 deg)
        // P1->P2: Fall 6 over 10 (~-31 deg) -> Turn ~47 deg
        // P2->P3: Rise 3 over 10 (~16 deg)  -> Turn ~47 deg
        // Total: ~94 degrees.
        // Threshold is 60..120 -> Depth 1
        let c = CubicBez::new((0.0, 0.0), (10.0, 3.0), (20.0, -3.0), (30.0, 0.0));
        assert_eq!(calculate_dynamic_depth(c), 1);
    }

    #[test]
    fn test_depth_degenerate_handles() {
        // P0==P1, so we measure angle P1->P2 vs P2->P3
        // 90 degree turn.
        let c = CubicBez::new((0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (10.0, 10.0));
        assert_eq!(calculate_dynamic_depth(c), 1);
    }
}
