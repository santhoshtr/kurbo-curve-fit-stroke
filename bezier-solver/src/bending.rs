use kurbo::{CubicBez, ParamCurveCurvature, Point};

/// Calculate bending energy using numerical integration
pub fn bending_energy(cubic_bez: CubicBez, num_samples: usize) -> f64 {
    if num_samples < 2 {
        return 0.0;
    }

    let mut energy = 0.0;
    let dt = 1.0 / (num_samples - 1) as f64;

    for i in 0..num_samples {
        let t = i as f64 * dt;
        let curvature = cubic_bez.curvature(t);
        let d1 = first_derivative(&cubic_bez, t);
        let speed = (d1.x * d1.x + d1.y * d1.y).sqrt();

        // Simpson's rule weights
        let weight = if i == 0 || i == num_samples - 1 {
            1.0
        } else if i % 2 == 1 {
            4.0
        } else {
            2.0
        };

        energy += weight * curvature * curvature * speed;
    }

    energy * dt / 3.0
}

pub fn first_derivative(cubic_bez: &CubicBez, t: f64) -> Point {
    let t2 = t * t;
    let mt = 1.0 - t;
    let mt2 = mt * mt;

    Point::new(
        3.0 * (-mt2 * cubic_bez.p0.x + mt2 * cubic_bez.p1.x - 2.0 * mt * t * cubic_bez.p1.x
            + 2.0 * mt * t * cubic_bez.p2.x
            - t2 * cubic_bez.p2.x
            + t2 * cubic_bez.p3.x),
        3.0 * (-mt2 * cubic_bez.p0.y + mt2 * cubic_bez.p1.y - 2.0 * mt * t * cubic_bez.p1.y
            + 2.0 * mt * t * cubic_bez.p2.y
            - t2 * cubic_bez.p2.y
            + t2 * cubic_bez.p3.y),
    )
}

/// Second derivative at parameter t
pub fn second_derivative(cubic_bez: &CubicBez, t: f64) -> Point {
    let mt = 1.0 - t;

    Point::new(
        6.0 * (mt * cubic_bez.p0.x - 2.0 * mt * cubic_bez.p1.x
            + mt * cubic_bez.p2.x
            + t * cubic_bez.p1.x
            - 2.0 * t * cubic_bez.p2.x
            + t * cubic_bez.p3.x),
        6.0 * (mt * cubic_bez.p0.y - 2.0 * mt * cubic_bez.p1.y
            + mt * cubic_bez.p2.y
            + t * cubic_bez.p1.y
            - 2.0 * t * cubic_bez.p2.y
            + t * cubic_bez.p3.y),
    )
}
