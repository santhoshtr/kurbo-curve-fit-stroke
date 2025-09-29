use kurbo::{BezPath, Point};

use crate::{ControlPoint, InputPoint, TwoParamCurve, two_param_spline::TwoParamSpline};

pub struct Spline {
    ctrl_pts: Vec<ControlPoint>,
    is_closed: bool,
}

impl Spline {
    pub fn new(input_points: Vec<InputPoint>, is_closed: bool) -> Self {
        let ctrl_pts = input_points
            .into_iter()
            .map(|p| ControlPoint::new(Point::new(p.x, p.y), p.point_type))
            .collect();

        Self {
            ctrl_pts,
            is_closed,
        }
    }

    pub fn solve(&mut self, curve: &TwoParamCurve) -> Result<(), String> {
        if self.ctrl_pts.len() < 2 {
            return Err("Need at least 2 points".to_string());
        }

        // For now, treat all as one smooth segment
        let points: Vec<Point> = self.ctrl_pts.iter().map(|cp| cp.pt).collect();
        let mut spline = TwoParamSpline::new(points);
        spline.initial_ths();

        // Solve with iterations
        for i in 0..10 {
            let err = spline.iter_solver(i, curve);
            if err < 1e-6 {
                break;
            }
        }

        // Store results back (simplified for now)
        for (i, cp) in self.ctrl_pts.iter_mut().enumerate() {
            if i < spline.ths.len() {
                cp.l_th = Some(spline.ths[i]);
                cp.r_th = Some(spline.ths[i]);
            }
        }

        Ok(())
    }

    pub fn render(&self, curve: &TwoParamCurve) -> BezPath {
        let points: Vec<Point> = self.ctrl_pts.iter().map(|cp| cp.pt).collect();
        let mut spline = TwoParamSpline::new(points);

        // Use stored tangents if available
        for (i, cp) in self.ctrl_pts.iter().enumerate() {
            if i < spline.ths.len() {
                if let Some(th) = cp.l_th {
                    spline.ths[i] = th;
                }
            }
        }

        spline.render_svg(curve)
    }
}
