use std::f64::consts::PI;

use kurbo::{CubicBez, Point};

use crate::bending::bending_energy;

mod bending;

/// Solver for optimal Bézier curve parameters
pub struct BezierSolver {
    max_iterations: usize,
    tolerance: f64,
}

impl Default for BezierSolver {
    fn default() -> Self {
        Self::new(100, 1e-6)
    }
}

impl BezierSolver {
    /// Create a new solver
    ///
    /// # Arguments
    /// * `max_iterations` - Maximum optimization iterations
    /// * `tolerance` - Convergence tolerance for optimization
    pub fn new(max_iterations: usize, tolerance: f64) -> Self {
        let solver = Self {
            max_iterations,
            tolerance,
        };
        // solver.precompute_cache();
        solver
    }

    /// Get optimal delta values for given angles
    pub fn solve(&self, theta0: f64, theta1: f64) -> (f64, f64) {
        let epsilon = 1e-6; // Tolerance for "approximately equal"

        // Case 1: Horizontal tangents (θ₀ = θ₁ = 0)
        if theta0.abs() < epsilon && theta1.abs() < epsilon {
            return (1.0 / 3.0, 1.0 / 3.0);
        }

        // Case 2: Symmetric angles (θ₁ = -θ₀)
        if (theta0 + theta1).abs() < epsilon {
            // For small angles, use the analytical approximation
            if theta0.abs() < PI / 4.0 {
                // Less than 45 degrees
                let delta = (4.0 / 3.0) * (theta0 / 4.0).tan();
                return (delta.abs(), delta.abs());
            }
            // For larger symmetric angles, use a modified approach
            else {
                let avg_angle = theta0.abs();
                // Use a more robust formula for larger angles
                let delta =
                    (2.0 / 3.0) * (avg_angle / 2.0).sin() / (1.0 + (avg_angle / PI).powi(2));
                return (delta, delta);
            }
        }

        // Case 3: Nearly horizontal (small angles)
        if theta0.abs() < epsilon / 10.0 && theta1.abs() < epsilon / 10.0 {
            return (1.0 / 3.0, 1.0 / 3.0);
        }

        // Case 4: One angle is zero (semi-horizontal)
        if theta0.abs() < epsilon {
            let delta1 = self.single_angle_approximation(theta1);
            return (1.0 / 3.0, delta1);
        }
        if theta1.abs() < epsilon {
            let delta0 = self.single_angle_approximation(theta0);
            return (delta0, 1.0 / 3.0);
        }

        self.optimize_bending_energy(theta0, theta1)
    }

    /// Approximation for cases where only one angle is non-zero
    fn single_angle_approximation(&self, theta: f64) -> f64 {
        if theta.abs() < PI / 6.0 {
            // Less than 30 degrees
            // Linear approximation for small angles
            (1.0 / 3.0) * (1.0 + theta.abs() / PI)
        } else {
            // More complex approximation for larger angles
            let normalized_angle = (theta.abs() / PI).min(1.0);
            (1.0 / 3.0) * (1.0 + 1.5 * normalized_angle)
        }
    }

    /// Optimize bending energy for given angles
    fn optimize_bending_energy(&self, theta0: f64, theta1: f64) -> (f64, f64) {
        // Initial guess based on geometric heuristic
        let mut delta0 = self.initial_guess(theta0);
        let mut delta1 = self.initial_guess(theta1);

        // Gradient descent optimization
        let mut learning_rate = 0.1;
        let mut prev_energy = f64::INFINITY;

        for iteration in 0..self.max_iterations {
            let energy = self.evaluate_energy(theta0, theta1, delta0, delta1);

            // Check convergence
            if (prev_energy - energy).abs() < self.tolerance {
                break;
            }

            // Compute gradients numerically
            let eps = 1e-6;
            let grad_delta0 = (self.evaluate_energy(theta0, theta1, delta0 + eps, delta1)
                - self.evaluate_energy(theta0, theta1, delta0 - eps, delta1))
                / (2.0 * eps);

            let grad_delta1 = (self.evaluate_energy(theta0, theta1, delta0, delta1 + eps)
                - self.evaluate_energy(theta0, theta1, delta0, delta1 - eps))
                / (2.0 * eps);

            // Update parameters
            delta0 -= learning_rate * grad_delta0;
            delta1 -= learning_rate * grad_delta1;

            // Clamp to reasonable bounds
            delta0 = delta0.max(0.01).min(2.0);
            delta1 = delta1.max(0.01).min(2.0);

            // Adaptive learning rate
            if energy > prev_energy {
                learning_rate *= 0.5;
            } else if iteration % 10 == 0 {
                learning_rate *= 1.1;
            }

            prev_energy = energy;
        }

        (delta0, delta1)
    }

    /// Initial guess for delta based on angle
    fn initial_guess(&self, theta: f64) -> f64 {
        if theta.abs() < 0.1 {
            1.0 / 3.0
        } else {
            let factor = (1.0 + theta.abs() / PI).min(2.0);
            factor / 3.0
        }
    }

    /// Evaluate bending energy for given parameters
    fn evaluate_energy(&self, theta0: f64, theta1: f64, delta0: f64, delta1: f64) -> f64 {
        let curve = self.create_curve(theta0, theta1, delta0, delta1);
        bending_energy(curve, 50)
    }

    /// Create Bézier curve with given parameters
    fn create_curve(&self, theta0: f64, theta1: f64, delta0: f64, delta1: f64) -> CubicBez {
        CubicBez::new(
            Point { x: 0.0, y: 0.0 },
            Point {
                x: delta0 * theta0.cos(),
                y: delta0 * theta0.sin(),
            },
            Point {
                x: 1.0 - delta1 * theta1.cos(),
                y: delta1 * theta1.sin(),
            },
            Point { x: 1.0, y: 0.0 },
        )
    }

    /// Create final Bézier curve with optimal parameters
    pub fn create_optimal_curve(&self, theta0: f64, theta1: f64) -> CubicBez {
        let (delta0, delta1) = self.solve(theta0, theta1);
        self.create_curve(theta0, theta1, delta0, delta1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use kurbo::ParamCurve;
    #[test]
    fn test_bezier_evaluation() {
        let curve = CubicBez::new(
            Point { x: 0.0, y: 0.0 },
            Point { x: 1.0, y: 1.0 },
            Point { x: 2.0, y: 1.0 },
            Point { x: 3.0, y: 0.0 },
        );

        let start = curve.eval(0.0);
        let end = curve.eval(1.0);

        assert!((start.x - 0.0).abs() < 1e-10);
        assert!((start.y - 0.0).abs() < 1e-10);
        assert!((end.x - 3.0).abs() < 1e-10);
        assert!((end.y - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_solver() {
        let solver = BezierSolver::new(50, 1e-4); // Coarse settings for fast test

        let theta0 = PI / 4.0; // 45 degrees
        let theta1 = -PI / 6.0; // -30 degrees

        let (delta0, delta1) = solver.solve(theta0, theta1);

        assert!(delta0 > 0.0 && delta0 < 2.0);
        assert!(delta1 > 0.0 && delta1 < 2.0);

        let curve = solver.create_optimal_curve(theta0, theta1);
        let energy = bending_energy(curve, 50);

        assert!(energy > 0.0);
        println!(
            "Optimal deltas: ({:.4}, {:.4}), Energy: {:.6}",
            delta0, delta1, energy
        );
    }
}
