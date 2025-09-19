use bezier_solver::BezierSolver;
use kurbo::{ParamCurve, Point};
use std::f64::consts::PI;
mod bending;

fn main() {
    // Create solver with default settings
    let solver = BezierSolver::default();

    // Example: Create curve from (0,0) to (1,0) with 45° and -30° angles
    let theta0 = PI / 4.0; // 45 degrees
    let theta1 = -PI / 6.0; // -30 degrees
    let theta1 = PI / 4.0; // 45 degrees

    let (delta0, delta1) = solver.solve(theta0, theta1);
    println!(
        "Optimal deltas for θ0={:.2}°, θ1={:.2}°: δ0={:.2}, δ1={:.2}",
        theta0.to_degrees(),
        theta1.to_degrees(),
        delta0,
        delta1
    );
    println!(
        "Control points for Cubic bezier: p0={:.2}, p1={:.2}: p2={:.2}, p3={:.2}",
        Point::new(0., 0.),
        Point::new(delta0 * theta0.cos(), delta0 * theta0.sin()),
        Point::new(1. - delta1 * theta1.cos(), delta1 * theta1.sin()),
        Point::new(1., 0.)
    );
    // Create the optimal curve
    let curve = solver.create_optimal_curve(theta0, theta1);
    let energy = bending::bending_energy(curve, 100);
    println!("Bending energy: {:.6}", energy);

    // Sample some points on the curve
    println!("\nCurve points:");
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        let point = curve.eval(t);
        println!("t={:.1}: ({:.2}, {:.2})", t, point.x, point.y);
    }
}
