use kurbo::{CubicBez, Point};

use crate::CurvatureResult;

trait CubicBezExt {
    fn derivative(&self, t: f64) -> Point;
    fn deriv2(&self, t: f64) -> Point;
}

impl CubicBezExt for CubicBez {
    fn derivative(&self, t: f64) -> Point {
        let mt = 1.0 - t;
        let c0 = -3.0 * mt * mt;
        let c3 = 3.0 * t * t;
        let c1 = -6.0 * t * mt - c0;
        let c2 = 6.0 * t * mt - c3;

        // Get control points
        let p0 = self.p0;
        let p1 = self.p1;
        let p2 = self.p2;
        let p3 = self.p3;

        let x = c0 * p0.x + c1 * p1.x + c2 * p2.x + c3 * p3.x;
        let y = c0 * p0.y + c1 * p1.y + c2 * p2.y + c3 * p3.y;

        Point::new(x, y)
    }

    fn deriv2(&self, t: f64) -> Point {
        let mt = 1.0 - t;
        let c0 = 6.0 * mt;
        let c3 = 6.0 * t;
        let c1 = 6.0 - 18.0 * mt;
        let c2 = 6.0 - 18.0 * t;

        // Get control points
        let p0 = self.p0;
        let p1 = self.p1;
        let p2 = self.p2;
        let p3 = self.p3;

        let x = c0 * p0.x + c1 * p1.x + c2 * p2.x + c3 * p3.x;
        let y = c0 * p0.y + c1 * p1.y + c2 * p2.y + c3 * p3.y;

        Point::new(x, y)
    }
}

pub struct TwoParamCurve;

impl TwoParamCurve {
    pub fn new() -> Self {
        Self
    }

    /// Generate control points for cubic bezier given tangent angles
    pub fn render(&self, th0: f64, th1: f64) -> Vec<Point> {
        let coords = self.my_cubic(th0, th1);
        vec![
            Point::new(coords[2], coords[3]), // first control point
            Point::new(coords[4], coords[5]), // second control point
        ]
    }

    /// Compute curvature at endpoints
    pub fn compute_curvature(&self, th0: f64, th1: f64) -> CurvatureResult {
        let coords = self.my_cubic(th0, th1);
        let cb = CubicBez::new(
            Point::new(coords[0], coords[1]),
            Point::new(coords[2], coords[3]),
            Point::new(coords[4], coords[5]),
            Point::new(coords[6], coords[7]),
        );

        let ak0 = self.compute_curvature_at_t(&cb, 0.0, th0);
        let ak1 = self.compute_curvature_at_t(&cb, 1.0, -th1);

        CurvatureResult { ak0, ak1 }
    }

    /// Generate cubic bezier coefficients
    fn my_cubic(&self, th0: f64, th1: f64) -> [f64; 8] {
        let len0 = self.my_cubic_len(th0, th1);
        let len1 = self.my_cubic_len(th1, th0);

        [
            0.0,
            0.0, // start point
            th0.cos() * len0,
            th0.sin() * len0, // first control
            1.0 - th1.cos() * len1,
            th1.sin() * len1, // second control
            1.0,
            0.0, // end point
        ]
    }

    fn my_cubic_len(&self, th0: f64, th1: f64) -> f64 {
        let offset = 0.3 * (th1 * 2.0 - 0.4 * (th1 * 2.0).sin()).sin();
        let scale = 1.0 / (3.0 * 0.8);
        scale * ((th0 - offset).cos() - 0.2 * (3.0 * (th0 - offset)).cos())
    }

    fn compute_curvature_at_t(&self, cb: &CubicBez, t: f64, th: f64) -> f64 {
        let d = cb.derivative(t);
        let d2 = cb.deriv2(t);
        let c = th.cos();
        let s = th.sin();
        let d2_cross = d2.y * c - d2.x * s;
        let d_dot = d.x * c + d.y * s;
        (d2_cross / (d_dot * d_dot.abs())).atan2(d_dot * d_dot.abs())
    }

    pub fn endpoint_tangent(&self, th: f64) -> f64 {
        0.5 * (2.0 * th).sin()
    }
}
