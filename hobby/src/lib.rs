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
        return vec![(points[0], c1, c2, points[1])];
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
        segments.push((points[k], c1, c2, points[k + 1]));
    }
    segments
}

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

        segments.push((points[k], c1, c2, points[next_k]));
    }
    return segments;
}
