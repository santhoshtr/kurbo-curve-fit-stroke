use kurbo::{BezPath, Point};

use crate::{CurvatureResult, SegmentParams, TwoParamCurve, mod2pi};

pub struct TwoParamSpline {
    ctrl_pts: Vec<Point>,
    pub(crate) ths: Vec<f64>, // tangent angles at each point
    start_th: Option<f64>,
    end_th: Option<f64>,
    is_closed: bool,
}

impl TwoParamSpline {
    pub fn new(ctrl_pts: Vec<Point>, is_closed: bool) -> Self {
        let n = ctrl_pts.len();
        Self {
            ctrl_pts,
            ths: vec![0.0; n],
            start_th: None,
            end_th: None,
            is_closed,
        }
    }

    pub fn set_start_tangent(&mut self, th: Option<f64>) {
        self.start_th = th;
    }

    pub fn set_end_tangent(&mut self, th: Option<f64>) {
        self.end_th = th;
    }

    /// Initialize tangent angles based on control polygon
    pub fn initial_ths(&mut self) {
        let n = self.ctrl_pts.len();
        if n < 2 {
            return;
        }

        for i in 1..n - 1 {
            let p0 = self.ctrl_pts[i - 1];
            let p1 = self.ctrl_pts[i];
            let p2 = self.ctrl_pts[i + 1];

            let dx0 = p1.x - p0.x;
            let dy0 = p1.y - p0.y;
            let l0 = (dx0 * dx0 + dy0 * dy0).sqrt();

            let dx1 = p2.x - p1.x;
            let dy1 = p2.y - p1.y;
            let l1 = (dx1 * dx1 + dy1 * dy1).sqrt();

            let th0 = dy0.atan2(dx0);
            let th1 = dy1.atan2(dx1);
            let bend = mod2pi(th1 - th0);
            let th = mod2pi(th0 + bend * l0 / (l0 + l1));

            self.ths[i] = th;

            if i == 1 {
                self.ths[0] = th0;
            }
            if i == n - 2 {
                self.ths[n - 1] = th1;
            }
        }

        if let Some(start_th) = self.start_th {
            self.ths[0] = start_th;
        }
        if let Some(end_th) = self.end_th {
            self.ths[n - 1] = end_th;
        }
    }

    /// Get tangent angles and chord length for segment i
    pub fn get_ths(&self, i: usize) -> SegmentParams {
        let p0 = self.ctrl_pts[i];
        let p1 = self.ctrl_pts[i + 1];
        let dx = p1.x - p0.x;
        let dy = p1.y - p0.y;
        let th = dy.atan2(dx);
        let th0 = mod2pi(self.ths[i] - th);
        let th1 = mod2pi(th - self.ths[i + 1]);
        let chord = (dx * dx + dy * dy).sqrt();

        SegmentParams { th0, th1, chord }
    }

    /// Perform one iteration of the curve fitting solver
    pub fn iter_solver(&mut self, iter: usize, curve: &TwoParamCurve) -> f64 {
        let n = self.ctrl_pts.len();
        if n < 3 {
            return 0.0;
        }

        // Fix endpoint tangents
        if self.start_th.is_none() {
            let ths0 = self.get_ths(0);
            self.ths[0] += curve.endpoint_tangent(ths0.th1) - ths0.th0;
        }

        if self.end_th.is_none() {
            let ths0 = self.get_ths(n - 2);
            self.ths[n - 1] -= curve.endpoint_tangent(ths0.th0) - ths0.th1;
        }

        // Correction to match start/end tangents on closed curves
        if self.is_closed && self.start_th.is_none() && self.end_th.is_none() {
            let avgth = (self.ths[0] + self.ths[n - 1]) / 2.0;
            self.ths[0] = avgth;
            self.ths[n - 1] = avgth;
        }

        let mut abs_err = 0.0;
        let mut corrections = vec![0.0; n - 2];

        let mut ths0 = self.get_ths(0);
        let mut ak0 = curve.compute_curvature(ths0.th0, ths0.th1);

        for i in 0..n - 2 {
            let ths1 = self.get_ths(i + 1);
            let ak1 = curve.compute_curvature(ths1.th0, ths1.th1);

            let err = self.compute_curvature_error(&ths0, &ak0, &ths1, &ak1);
            abs_err += err.abs();

            // Numerical derivative for Newton step
            let epsilon = 1e-3;
            let ak0_p = curve.compute_curvature(ths0.th0, ths0.th1 + epsilon);
            let ak1_p = curve.compute_curvature(ths1.th0 - epsilon, ths1.th1);
            let err_p = self.compute_curvature_error(&ths0, &ak0_p, &ths1, &ak1_p);
            let derr = (err_p - err) / epsilon;

            if derr.abs() > 1e-12 {
                corrections[i] = err / derr;
            }

            ths0 = ths1;
            ak0 = ak1;
        }

        // Apply corrections with damping
        let scale = (0.25 * (iter + 1) as f64).tanh();
        for i in 0..n - 2 {
            self.ths[i + 1] += scale * corrections[i];
        }

        abs_err
    }

    fn compute_curvature_error(
        &self,
        ths0: &SegmentParams,
        ak0: &CurvatureResult,
        ths1: &SegmentParams,
        ak1: &CurvatureResult,
    ) -> f64 {
        let ch0 = ths0.chord.sqrt();
        let ch1 = ths1.chord.sqrt();
        let a0 = (ak0.ak1.sin() * ch1).atan2(ak0.ak1.cos() * ch0);
        let a1 = (ak1.ak0.sin() * ch0).atan2(ak1.ak0.cos() * ch1);
        a0 - a1
    }

    pub fn render_svg(&self, curve: &TwoParamCurve) -> BezPath {
        let mut path = BezPath::new();
        if self.ctrl_pts.is_empty() {
            return path;
        }

        path.move_to(self.ctrl_pts[0]);

        for i in 0..self.ctrl_pts.len() - 1 {
            let ths = self.get_ths(i);
            let render = curve.render(ths.th0, ths.th1, None, None);

            let p0 = self.ctrl_pts[i];
            let p1 = self.ctrl_pts[i + 1];
            let dx = p1.x - p0.x;
            let dy = p1.y - p0.y;

            let mut control_points = Vec::new();
            for pt in render {
                let x = p0.x + dx * pt.x - dy * pt.y;
                let y = p0.y + dy * pt.x + dx * pt.y;
                control_points.push(Point::new(x, y));
            }

            if control_points.len() >= 2 {
                path.curve_to(control_points[0], control_points[1], p1);
            } else {
                path.line_to(p1);
            }
        }

        path
    }
}
