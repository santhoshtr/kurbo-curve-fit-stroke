use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;

use crate::point::Point;
pub mod point;
// The Point struct from above goes here...

/// Represents the four points defining a cubic Bézier curve.
pub type BezierSegment = (Point, Point, Point, Point);

/// Implements Hobby's algorithm for finding optimal Bézier control points.
///
/// # Arguments
/// * `points` - The points the curve should pass through.
/// * `tensions` - Optional tension at each point. Defaults to 1.0.
/// * `cyclic` - Whether the curve is a closed loop.
/// * `entry_angles` - A map from a point's index to its desired entry angle in degrees.
/// * `exit_angles` - A map from a point's index to its desired exit angle in degrees.
///
/// # Returns
/// A vector of `BezierSegment`s.
pub fn hobby(
    points: &[Point],
    tensions: Option<&[f64]>,
    cyclic: bool,
    entry_angles: Option<&HashMap<usize, f64>>,
    exit_angles: Option<&HashMap<usize, f64>>,
) -> Vec<BezierSegment> {
    let n = points.len();
    if n < 2 {
        return vec![];
    }

    let tensions = tensions.map(|t| t.to_vec()).unwrap_or_else(|| vec![1.0; n]);

    // Pre-computation: chord vectors, lengths, and angles
    let d: Vec<Point> = points.windows(2).map(|p| p[1] - p[0]).collect();
    let l: Vec<f64> = d.iter().map(|v| v.norm()).collect();
    let psi: Vec<f64> = d.iter().map(|v| v.angle().to_degrees()).collect();

    if n == 2 {
        // Handle the simple two-point case
        let tension_a = tensions[0];
        let tension_b = tensions[1];
        let theta = exit_angles
            .and_then(|h| h.get(&0))
            .map_or(0.0, |a| a - psi[0]);
        let phi = entry_angles
            .and_then(|h| h.get(&1))
            .map_or(0.0, |a| psi[0] - a);

        let c1_angle = (psi[0] + theta).to_radians();
        let c1 =
            points[0] + Point::new(c1_angle.cos(), c1_angle.sin()) * (l[0] / (3.0 * tension_a));

        let c2_angle = (psi[0] - phi).to_radians();
        let c2 =
            points[1] - Point::new(c2_angle.cos(), c2_angle.sin()) * (l[0] / (3.0 * tension_b));

        return vec![(points[0], c1, c2, points[1])];
    }

    // For non-cyclic curves with more than 2 points
    // We solve a tridiagonal system for the internal points
    let size = n - 2;
    let mut a = DMatrix::<f64>::zeros(size, size);
    let mut b = DVector::<f64>::zeros(size);

    for k in 0..size {
        let i = k + 1; // Actual point index
        let alpha = tensions[i + 1] / tensions[i] * l[i] / l[i - 1];
        let beta = -2.0 - 2.0 * alpha;

        // Main diagonal
        a[(k, k)] = beta;
        // Off-diagonals
        if k > 0 {
            a[(k, k - 1)] = 1.0;
        }
        if k < size - 1 {
            a[(k, k + 1)] = alpha;
        }

        b[k] = -3.0 * alpha * (psi[i] - psi[i - 1]);
    }

    // Apply angle constraints by modifying the 'b' vector
    // An exit angle at points[0] becomes a known `theta[0]`
    if let Some(angle) = exit_angles.and_then(|h| h.get(&0)) {
        let theta0 = angle - psi[0];
        b[0] -= theta0;
    }

    // An entry angle at points[n-1] becomes a known `phi[n-1]`
    if let Some(angle) = entry_angles.and_then(|h| h.get(&(n - 1))) {
        let phi_n_minus_1 = psi[n - 2] - angle;
        let alpha_final = tensions[n - 1] / tensions[n - 2] * l[n - 2] / l[n - 3];
        b[size - 1] -= alpha_final * phi_n_minus_1;
    }

    // Solve the system A * theta = b for the internal theta values
    let lu = a.lu();
    let internal_thetas = lu.solve(&b).expect("Linear system has no solution.");

    let mut theta = vec![0.0; n];
    let mut phi = vec![0.0; n];

    // Populate the full theta vector
    theta[1..n - 1].copy_from_slice(internal_thetas.as_slice());
    if let Some(angle) = exit_angles.and_then(|h| h.get(&0)) {
        theta[0] = angle - psi[0];
    }
    if let Some(angle) = entry_angles.and_then(|h| h.get(&(n - 1))) {
        // This theta is determined by the final phi
    } else {
        theta[n - 1] = -phi[n - 1]; // Default condition
    }

    // Calculate phi values from the solved thetas
    for k in 0..size {
        let i = k + 1;
        let alpha = tensions[i + 1] / tensions[i] * l[i] / l[i - 1];
        let beta = -2.0 - 2.0 * alpha;
        phi[i] = -(alpha * theta[i + 1] + beta * theta[i] + 3.0 * alpha * (psi[i] - psi[i - 1]));
    }
    if let Some(angle) = entry_angles.and_then(|h| h.get(&(n - 1))) {
        phi[n - 1] = psi[n - 2] - angle;
    }

    // Now, build the Bézier segments
    let mut segments = Vec::new();
    for k in 0..(n - 1) {
        let c1_angle = (psi[k] + theta[k]).to_radians();
        let c1 =
            points[k] + Point::new(c1_angle.cos(), c1_angle.sin()) * (l[k] / (3.0 * tensions[k]));

        let c2_angle = (psi[k] - phi[k + 1]).to_radians();
        let c2 = points[k + 1]
            - Point::new(c2_angle.cos(), c2_angle.sin()) * (l[k] / (3.0 * tensions[k + 1]));

        segments.push((points[k], c1, c2, points[k + 1]));
    }

    segments
}
