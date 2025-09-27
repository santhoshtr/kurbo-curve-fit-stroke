//! # Hobby Curve Library
//!
//! This library implements John Hobby's algorithm for generating smooth curves through a sequence of points.
//! The algorithm finds optimal cubic Bézier control points that create aesthetically pleasing curves
//! with continuous curvature.
//!
//! ## Features
//!
//! - Generate smooth cubic Bézier curves through any sequence of points
//! - Support for both open and closed (cyclic) paths
//! - Configurable tension parameters for curve tightness control
//! - Directional constraints for entry and exit angles at specific points
//! - Automatic handling of coordinate system transformations
//!
//! ## Basic Usage
//!
//! ```rust
//! use hobby::{hobby, point::Point};
//!
//! let points = vec![
//!     Point::new(0.0, 0.0),
//!     Point::new(100.0, 50.0),
//!     Point::new(200.0, 0.0),
//! ];
//!
//! let segments = hobby(&points, None, false, None, None);
//!
//! // Each segment is a BezierSegment struct with named fields
//! for segment in segments {
//!     println!("Curve from {} to {}", segment.start, segment.end);
//!     println!("Control points: {}, {}", segment.control1, segment.control2);
//! }
//! ```
//!
//! ## Advanced Usage with Angle Constraints
//!
//! ```rust
//! use hobby::{hobby, point::Point};
//! use std::collections::HashMap;
//!
//! let points = vec![
//!     Point::new(0.0, 0.0),
//!     Point::new(100.0, 100.0),
//!     Point::new(200.0, 0.0),
//! ];
//!
//! let mut exit_angles = HashMap::new();
//! exit_angles.insert(0, 45.0); // Exit at 45 degrees from first point
//!
//! let mut entry_angles = HashMap::new();
//! entry_angles.insert(2, -45.0); // Enter last point at -45 degrees
//!
//! let segments = hobby(&points, None, false, Some(&entry_angles), Some(&exit_angles));
//! ```

use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;

use crate::{point::Point, segment::BezierSegment};
pub mod point;
pub mod segment;
#[cfg(feature = "wasm")]
pub mod wasm;

/// Implements Hobby's algorithm for finding optimal Bézier control points.
///
/// This function generates a sequence of cubic Bézier curve segments that pass through
/// all the given points with smooth transitions and optimal curvature distribution.
///
/// # Arguments
///
/// * `points` - The points the curve should pass through. Must contain at least 2 points.
/// * `tensions` - Optional tension at each point. Higher values create tighter curves.
///               If `None`, defaults to 1.0 for all points. If provided, must have same length as `points`.
/// * `cyclic` - Whether the curve is a closed loop (connects last point back to first).
/// * `entry_angles` - A map from point index to desired entry angle in degrees.
///                   Entry angle is the direction the curve approaches the point from.
/// * `exit_angles` - A map from point index to desired exit angle in degrees.
///                  Exit angle is the direction the curve leaves the point toward.
///
/// # Returns
///
/// A vector of `BezierSegment` structs representing the cubic Bézier curves.
/// Each segment contains four `Point` fields: `start`, `control1`, `control2`, and `end`.
/// For non-cyclic curves with n points, returns n-1 segments.
/// For cyclic curves with n points, returns n segments.
///
/// # Panics
///
/// Panics if the linear system for finding optimal angles has no solution.
/// This should not happen with valid input data.
///
/// # Examples
///
/// ## Simple curve through three points
///
/// ```rust
/// use hobby::{hobby, point::Point};
///
/// let points = vec![
///     Point::new(0.0, 0.0),
///     Point::new(50.0, 100.0),
///     Point::new(100.0, 0.0),
/// ];
///
/// let segments = hobby(&points, None, false, None, None);
/// assert_eq!(segments.len(), 2); // Two segments for three points
///
/// // Access the segment data
/// let first_segment = &segments[0];
/// println!("First segment starts at: {}", first_segment.start);
/// println!("First control point: {}", first_segment.control1);
/// println!("Second control point: {}", first_segment.control2);
/// println!("First segment ends at: {}", first_segment.end);
/// ```
///
/// ## Closed curve with tension control
///
/// ```rust
/// use hobby::{hobby, point::Point};
///
/// let points = vec![
///     Point::new(0.0, 0.0),
///     Point::new(100.0, 0.0),
///     Point::new(50.0, 100.0),
/// ];
///
/// let tensions = vec![2.0, 1.0, 0.5]; // Vary tension at each point
/// let segments = hobby(&points, Some(&tensions), true, None, None);
/// assert_eq!(segments.len(), 3); // Three segments for closed triangle
/// ```
pub fn hobby(
    points: &[Point],
    tensions: Option<&[f64]>,
    cyclic: bool,
    entry_angles: Option<&HashMap<usize, f64>>,
    exit_angles: Option<&HashMap<usize, f64>>,
) -> Vec<BezierSegment> {
    if cyclic {
        return hobby_cyclic(points, tensions, entry_angles, exit_angles);
    }
    let n = points.len();
    if n < 2 {
        return vec![];
    }

    let tensions = tensions.map(|t| t.to_vec()).unwrap_or_else(|| vec![1.0; n]);

    let d: Vec<Point> = points.windows(2).map(|p| p[1] - p[0]).collect();
    let l: Vec<f64> = d.iter().map(|v| v.norm()).collect();
    let psi: Vec<f64> = d.iter().map(|v| v.angle().to_degrees()).collect();

    if n == 2 {
        let theta0 = exit_angles
            .and_then(|h| h.get(&0))
            .map_or(0.0, |a| a - psi[0]);
        let phi1 = entry_angles
            .and_then(|h| h.get(&1))
            .map_or(0.0, |a| psi[0] - a);
        let c1_angle = (psi[0] + theta0).to_radians();
        let c1 =
            points[0] + Point::new(c1_angle.cos(), c1_angle.sin()) * (l[0] / (3.0 * tensions[0]));
        let c2_angle = (psi[0] - phi1).to_radians();
        let c2 =
            points[1] - Point::new(c2_angle.cos(), c2_angle.sin()) * (l[0] / (3.0 * tensions[1]));
        return vec![BezierSegment::new(points[0], c1, c2, points[1])];
    }

    let size = n - 2;
    let mut a = DMatrix::<f64>::zeros(size, size);
    let mut b = DVector::<f64>::zeros(size);
    let mut theta = vec![0.0; n];

    // Build the default system of equations for mock curvature continuity
    for k in 0..size {
        let i = k + 1; // Equation for point i
        let alpha = (tensions[i] / tensions[i + 1]) * (l[i - 1] / l[i]);
        let beta = -2.0 - 2.0 * alpha;

        a[(k, k)] = beta;
        if k > 0 {
            a[(k, k - 1)] = 1.0;
        }
        if k < size - 1 {
            a[(k, k + 1)] = alpha;
        }

        b[k] = -3.0 * alpha * (psi[i] - psi[i - 1]);
    }

    // --- Apply constraints by modifying the system ---

    // Handle endpoint exit angle
    if let Some(angle) = exit_angles.and_then(|h| h.get(&0)) {
        let known_theta0 = angle - psi[0];
        theta[0] = known_theta0;
        b[0] -= known_theta0;
    }

    // Handle endpoint entry angle
    if let Some(angle) = entry_angles.and_then(|h| h.get(&(n - 1))) {
        let known_phi_n1 = psi[n - 2] - angle;
        let alpha = (tensions[n - 2] / tensions[n - 1]) * (l[n - 3] / l[n - 2]);
        b[size - 1] -= alpha * known_phi_n1;
    }

    // Handle INTERIOR entry angles
    if let Some(angles) = entry_angles {
        for i in 1..n - 1 {
            if let Some(angle) = angles.get(&i) {
                let k = i - 1; // The matrix row for point i
                let known_phi = psi[i - 1] - angle;

                // This constraint REPLACES the default smoothness equation for point i.
                // It creates a new equation relating theta[i-1] and theta[i].
                let alpha = (tensions[i - 1] / tensions[i]) * (l[i] / l[i - 1]);
                let beta = (tensions[i + 1] / tensions[i]) * (l[i - 1] / l[i]);

                // Overwrite row k
                a.set_row(k, &DVector::zeros(size).transpose());
                if k > 0 {
                    a[(k, k - 1)] = alpha;
                } // Coeff for theta[i-1]
                a[(k, k)] = 1.0; // Coeff for theta[i]

                b[k] = -beta * known_phi - 3.0 * (psi[i] - psi[i - 1]);
            }
        }
    }

    // Handle INTERIOR exit angles (these take precedence over entry angles)
    if let Some(angles) = exit_angles {
        for i in 1..n - 1 {
            if let Some(angle) = angles.get(&i) {
                let k = i - 1;
                let known_theta = angle - psi[i];

                if k > 0 {
                    b[k - 1] -= a[(k - 1, k)] * known_theta;
                }
                if k < size - 1 {
                    b[k + 1] -= a[(k + 1, k)] * known_theta;
                }

                a.set_row(k, &DVector::zeros(size).transpose());
                a[(k, k)] = 1.0;
                b[k] = known_theta;
            }
        }
    }

    // Solve and compute final values
    let solution = a.lu().solve(&b).expect("Linear system has no solution.");
    theta[1..n - 1].copy_from_slice(solution.as_slice());

    let mut phi = vec![0.0; n];
    // Final calculation of phi values based on solved thetas
    phi[0] = 0.0; // Not used
    if let Some(angle) = entry_angles.and_then(|h| h.get(&(n - 1))) {
        phi[n - 1] = psi[n - 2] - angle;
    } else {
        phi[n - 1] = -theta[n - 1]; // Default end condition
    }

    for i in 1..n - 1 {
        if let Some(angle) = entry_angles.and_then(|h| h.get(&i)) {
            phi[i] = psi[i - 1] - angle;
        } else {
            let alpha = (tensions[i] / tensions[i + 1]) * (l[i - 1] / l[i]);
            let beta = -2.0 - 2.0 * alpha;
            phi[i] = -(beta * theta[i] + theta[i - 1] + 3.0 * alpha * (psi[i] - psi[i - 1]));
        }
    }

    // Construct the Bezier segments
    let mut segments = Vec::new();
    for k in 0..(n - 1) {
        let c1_angle = (psi[k] + theta[k]).to_radians();
        let c1 =
            points[k] + Point::new(c1_angle.cos(), c1_angle.sin()) * (l[k] / (3.0 * tensions[k]));
        let c2_angle = (psi[k] - phi[k + 1]).to_radians();
        let c2 = points[k + 1]
            - Point::new(c2_angle.cos(), c2_angle.sin()) * (l[k] / (3.0 * tensions[k + 1]));
        segments.push(BezierSegment::new(points[k], c1, c2, points[k + 1]));
    }
    segments
}

/// Internal function for handling cyclic (closed) curves.
///
/// This implements the cyclic version of Hobby's algorithm where the curve forms
/// a closed loop by connecting the last point back to the first point.
fn hobby_cyclic(
    points: &[Point],
    tensions: Option<&[f64]>,
    entry_angles: Option<&HashMap<usize, f64>>,
    exit_angles: Option<&HashMap<usize, f64>>,
) -> Vec<BezierSegment> {
    let n = points.len();
    if n < 2 {
        return vec![];
    }
    let tensions = tensions.map(|t| t.to_vec()).unwrap_or_else(|| vec![1.0; n]);
    let mut closed_points = points.to_vec();
    closed_points.push(points[0]); // Temporarily add start point at the end for chord calculation

    let d: Vec<Point> = closed_points.windows(2).map(|p| p[1] - p[0]).collect();
    let l: Vec<f64> = d.iter().map(|v| v.norm()).collect();
    let psi: Vec<f64> = d.iter().map(|v| v.angle().to_degrees()).collect();

    let mut a = DMatrix::<f64>::zeros(n, n);
    let mut b = DVector::<f64>::zeros(n);

    for k in 0..n {
        let prev = (k + n - 1) % n; // Wrap-around index for k-1
        let next = (k + 1) % n; // Wrap-around index for k+1

        let alpha = (tensions[k] / tensions[next]) * (l[prev] / l[k]);
        let beta = -2.0 - 2.0 * alpha;

        a[(k, prev)] = 1.0;
        a[(k, k)] = beta;
        a[(k, next)] = alpha;

        b[k] = -3.0 * alpha * (psi[k] - psi[prev]);
    }

    // Note: Applying angle constraints in a cyclic context is complex
    // and often omitted, as the loop's shape is determined by all points.
    // For simplicity, this implementation doesn't support angle constraints in cyclic mode.

    let theta = a.lu().solve(&b).expect("Cyclic system has no solution.");

    let mut phi = vec![0.0; n];
    for k in 0..n {
        let prev = (k + n - 1) % n;
        let next = (k + 1) % n;
        let alpha = (tensions[k] / tensions[next]) * (l[prev] / l[k]);
        let beta = -2.0 - 2.0 * alpha;
        phi[k] = -(beta * theta[k] + theta[prev] + 3.0 * alpha * (psi[k] - psi[prev]));
    }

    let mut segments = Vec::new();
    for k in 0..n {
        let next_k = (k + 1) % n;
        let c1_angle = (psi[k] + theta[k]).to_radians();
        let c1 =
            points[k] + Point::new(c1_angle.cos(), c1_angle.sin()) * (l[k] / (3.0 * tensions[k]));

        let c2_angle = (psi[k] - phi[next_k]).to_radians();
        let c2 = points[next_k]
            - Point::new(c2_angle.cos(), c2_angle.sin()) * (l[k] / (3.0 * tensions[next_k]));

        segments.push(BezierSegment::new(points[k], c1, c2, points[next_k]));
    }
    return segments;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// Helper function to check if two points are approximately equal
    fn points_approx_equal(p1: Point, p2: Point, tolerance: f64) -> bool {
        (p1.x - p2.x).abs() < tolerance && (p1.y - p2.y).abs() < tolerance
    }

    /// Helper function to check if a bezier segment passes through its endpoints
    fn check_endpoints(segment: &BezierSegment, start: Point, end: Point) -> bool {
        points_approx_equal(segment.start, start, 1e-10)
            && points_approx_equal(segment.end, end, 1e-10)
    }

    #[test]
    fn test_empty_points() {
        let points: Vec<Point> = vec![];
        let segments = hobby(&points, None, false, None, None);
        assert_eq!(segments.len(), 0);
    }

    #[test]
    fn test_single_point() {
        let points = vec![Point::new(0.0, 0.0)];
        let segments = hobby(&points, None, false, None, None);
        assert_eq!(segments.len(), 0);
    }

    #[test]
    fn test_two_points() {
        let points = vec![Point::new(0.0, 0.0), Point::new(100.0, 0.0)];
        let segments = hobby(&points, None, false, None, None);

        assert_eq!(segments.len(), 1);
        assert!(check_endpoints(&segments[0], points[0], points[1]));
    }

    #[test]
    fn test_three_points_horizontal_line() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(50.0, 0.0),
            Point::new(100.0, 0.0),
        ];
        let segments = hobby(&points, None, false, None, None);

        assert_eq!(segments.len(), 2);

        // Check that segments connect properly
        assert!(check_endpoints(&segments[0], points[0], points[1]));
        assert!(check_endpoints(&segments[1], points[1], points[2]));
    }

    #[test]
    fn test_three_points_triangle() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(200.0, 0.0),
        ];
        let segments = hobby(&points, None, false, None, None);

        assert_eq!(segments.len(), 2);

        // Verify endpoints
        assert!(check_endpoints(&segments[0], points[0], points[1]));
        assert!(check_endpoints(&segments[1], points[1], points[2]));

        // Check that control points are reasonable (not at infinity or NaN)
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_with_custom_tensions() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(50.0, 100.0),
            Point::new(100.0, 0.0),
        ];
        let tensions = vec![2.0, 1.0, 0.5];
        let segments = hobby(&points, Some(&tensions), false, None, None);

        assert_eq!(segments.len(), 2);

        // Verify all segments have finite control points
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_with_exit_angles() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(200.0, 100.0),
        ];

        let mut exit_angles = HashMap::new();
        exit_angles.insert(0, 45.0); // Exit first point at 45 degrees

        let segments = hobby(&points, None, false, None, Some(&exit_angles));

        assert_eq!(segments.len(), 2);

        // The first control point should reflect the 45-degree exit angle
        let first_segment = &segments[0];
        let control_direction = first_segment.control1 - first_segment.start;
        let angle = control_direction.angle().to_degrees();

        // Should be approximately 45 degrees (within tolerance for numerical precision)
        assert!((angle - 45.0).abs() < 1.0);
    }

    #[test]
    fn test_with_entry_angles() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(200.0, 0.0),
        ];

        let mut entry_angles = HashMap::new();
        entry_angles.insert(2, -45.0); // Enter last point at -45 degrees

        let segments = hobby(&points, None, false, Some(&entry_angles), None);

        assert_eq!(segments.len(), 2);

        // All control points should be finite
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_cyclic_curve() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(50.0, 100.0),
        ];
        let segments = hobby(&points, None, true, None, None);

        // For cyclic curve with 3 points, we should get 3 segments
        assert_eq!(segments.len(), 3);

        // Verify the cycle: last segment should end where first begins
        assert!(check_endpoints(&segments[2], points[2], points[0]));

        // Check all segments have finite control points
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_cyclic_square() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(0.0, 100.0),
        ];
        let segments = hobby(&points, None, true, None, None);

        assert_eq!(segments.len(), 4);

        // Verify each segment connects properly
        for i in 0..4 {
            let next_i = (i + 1) % 4;
            assert!(check_endpoints(&segments[i], points[i], points[next_i]));
        }
    }

    #[test]
    fn test_large_coordinate_values() {
        let points = vec![
            Point::new(1000000.0, 2000000.0),
            Point::new(1500000.0, 2500000.0),
            Point::new(2000000.0, 2000000.0),
        ];
        let segments = hobby(&points, None, false, None, None);

        assert_eq!(segments.len(), 2);

        // Should handle large coordinates without issues
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_negative_coordinates() {
        let points = vec![
            Point::new(-100.0, -50.0),
            Point::new(-50.0, 100.0),
            Point::new(100.0, -100.0),
        ];
        let segments = hobby(&points, None, false, None, None);

        assert_eq!(segments.len(), 2);

        // Should handle negative coordinates properly
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_very_close_points() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(0.001, 0.001),
            Point::new(1.0, 1.0),
        ];
        let segments = hobby(&points, None, false, None, None);

        assert_eq!(segments.len(), 2);

        // Should handle very close points without numerical issues
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }

    #[test]
    fn test_multiple_angle_constraints() {
        let points = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 50.0),
            Point::new(200.0, 0.0),
            Point::new(300.0, 100.0),
        ];

        let mut exit_angles = HashMap::new();
        exit_angles.insert(0, 30.0);
        exit_angles.insert(1, -60.0);

        let mut entry_angles = HashMap::new();
        entry_angles.insert(2, 120.0);
        entry_angles.insert(3, 0.0);

        let segments = hobby(
            &points,
            None,
            false,
            Some(&entry_angles),
            Some(&exit_angles),
        );

        assert_eq!(segments.len(), 3);

        // All segments should have finite control points
        for segment in &segments {
            assert!(segment.control1.x.is_finite() && segment.control1.y.is_finite());
            assert!(segment.control2.x.is_finite() && segment.control2.y.is_finite());
        }
    }
}
