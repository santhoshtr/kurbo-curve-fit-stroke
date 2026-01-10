use kurbo::{BezPath, Point};

use crate::{
    ControlPoint, InputPoint, PointType, TwoParamCurve, mod2pi, two_param_spline::TwoParamSpline,
};

pub struct Spline {
    ctrl_pts: Vec<ControlPoint>,
    is_closed: bool,
}

impl Spline {
    pub fn new(input_points: Vec<InputPoint>, is_closed: bool) -> Self {
        let ctrl_pts = input_points
            .into_iter()
            .map(|p| {
                ControlPoint::new(
                    Point::new(p.x, p.y),
                    p.point_type,
                    p.incoming_angle,
                    p.outgoing_angle,
                )
            })
            .collect();

        Self {
            ctrl_pts,
            is_closed,
        }
    }

    pub fn solve(&mut self) -> Result<(), String> {
        if self.ctrl_pts.len() < 2 {
            return Err("Need at least 2 points".to_string());
        }

        let start = self.start_ix();
        let length = self.ctrl_pts.len() - if self.is_closed { 0 } else { 1 };

        // Handle LineToCurve and CurveToLine point types by converting them to corners
        // with fixed theta from line direction
        // Thanks to https://github.com/terryspitz
        for i in 0..length {
            let actual_i = (i + start) % self.ctrl_pts.len();
            let pt_i = &self.ctrl_pts[actual_i];

            // Check if point needs line-based tangent calculation
            let needs_line_tangent =
                matches!(pt_i.ty, PointType::LineToCurve | PointType::CurveToLine);

            if needs_line_tangent && pt_i.lth.is_none() && pt_i.rth.is_none() {
                // Determine which adjacent point to use for line direction
                let adjacent_i = if matches!(pt_i.ty, PointType::LineToCurve) {
                    // LineToCurve: use previous point
                    if i == 0 && !self.is_closed {
                        continue; // No previous point available
                    }
                    let prev_i = if i == 0 {
                        self.ctrl_pts.len() - 1
                    } else {
                        i - 1
                    };
                    (prev_i + start) % self.ctrl_pts.len()
                } else {
                    // CurveToLine: use next point
                    if i + 1 >= length {
                        continue; // No next point available
                    }
                    (i + 1 + start) % self.ctrl_pts.len()
                };

                let pt_adjacent = &self.ctrl_pts[adjacent_i];

                // Calculate line direction
                let dx = pt_i.pt.x - pt_adjacent.pt.x;
                let dy = pt_i.pt.y - pt_adjacent.pt.y;
                let th = dy.atan2(dx);

                // Convert to corner with fixed tangent
                self.ctrl_pts[actual_i].ty = PointType::Corner;
                self.ctrl_pts[actual_i].lth = Some(th);
                self.ctrl_pts[actual_i].rth = Some(th);
            }
        }

        // Convert to corners if incoming and outgoing angles differ
        for i in 0..self.ctrl_pts.len() {
            let pt = &self.ctrl_pts[i];

            // If both angles are specified and they differ, treat as corner
            if let (Some(lth), Some(rth)) = (pt.lth, pt.rth) {
                const ANGLE_EPSILON: f64 = 1e-6; // ~0.00006 degrees in radians

                if (mod2pi(lth - rth)).abs() > ANGLE_EPSILON {
                    // Angles differ - convert to corner for non-smooth transition
                    self.ctrl_pts[i].ty = PointType::Corner;
                }
            }
        }

        let mut i = 0;

        while i < length {
            let pt_i = self.pt(i, start);
            let pt_i1 = self.pt(i + 1, start);

            // Check if this is a simple line segment (corner to corner with no explicit tangents)
            if (i + 1 == length || matches!(pt_i1.ty, PointType::Corner))
                && pt_i.rth.is_none()
                && pt_i1.lth.is_none()
            {
                // Simple line segment - set tangents to chord direction
                let dx = pt_i1.pt.x - pt_i.pt.x;
                let dy = pt_i1.pt.y - pt_i.pt.y;
                let th = dy.atan2(dx);

                // Modify the actual control points
                let actual_i = (i + start) % self.ctrl_pts.len();
                let actual_i1 = (i + 1 + start) % self.ctrl_pts.len();
                self.ctrl_pts[actual_i].r_th = Some(th);
                self.ctrl_pts[actual_i1].l_th = Some(th);

                i += 1;
            } else {
                // We have a curve segment - collect all points until next corner
                let mut inner_pts = vec![pt_i.pt];
                let mut j = i + 1;

                while j < length + 1 {
                    let pt_j = self.pt(j, start);
                    inner_pts.push(pt_j.pt);
                    j += 1;
                    if matches!(pt_j.ty, PointType::Corner) || pt_j.lth.is_some() {
                        break;
                    }
                }

                // Create and solve spline for this segment
                let mut inner = TwoParamSpline::new(inner_pts, self.is_closed);
                inner.set_start_tangent(self.pt(i, start).rth);
                inner.set_end_tangent(self.pt(j - 1, start).lth);

                let n_iter = 10;
                inner.initial_ths();
                let curve = TwoParamCurve::new();
                for k in 0..n_iter {
                    let err = inner.iter_solver(k, &curve);
                    if err < 1e-6 {
                        break;
                    }
                }

                // Store results back to control points
                for k in i..(j - 1) {
                    let actual_k = (k + start) % self.ctrl_pts.len();
                    let actual_k1 = (k + 1 + start) % self.ctrl_pts.len();

                    self.ctrl_pts[actual_k].r_th = Some(inner.ths[k - i]);
                    self.ctrl_pts[actual_k1].l_th = Some(inner.ths[k + 1 - i]);

                    // Also record curvatures for potential blending
                    let ths = inner.get_ths(k - i);
                    let ask = curve.compute_curvature(ths.th0, ths.th1);
                    self.ctrl_pts[actual_k].r_ak = Some(ask.ak0);
                    self.ctrl_pts[actual_k1].l_ak = Some(ask.ak1);
                }

                i = j - 1;
            }
        }

        Ok(())
    }

    pub fn render(&self) -> BezPath {
        let mut path = BezPath::new();
        if self.ctrl_pts.is_empty() {
            return path;
        }

        path.move_to(self.ctrl_pts[0].pt);

        let length = self.ctrl_pts.len() - if self.is_closed { 0 } else { 1 };

        for i in 0..length {
            let pt_i = self.pt(i, 0);
            let pt_i1 = self.pt(i + 1, 0);

            let dx = pt_i1.pt.x - pt_i.pt.x;
            let dy = pt_i1.pt.y - pt_i.pt.y;
            let chth = dy.atan2(dx);
            let chord = (dx * dx + dy * dy).sqrt();

            // Get tangent angles relative to chord
            let th0 = mod2pi(pt_i.r_th.unwrap_or(chth) - chth);
            let th1 = mod2pi(chth - pt_i1.l_th.unwrap_or(chth));

            // Apply curvature blending if available
            let k0 = pt_i.k_blend.map(|k| k * chord);
            let k1 = pt_i1.k_blend.map(|k| k * chord);

            let curve = TwoParamCurve::new();
            let render = curve.render(th0, th1, k0, k1);
            let mut control_points = Vec::new();
            for pt in render {
                let x = pt_i.pt.x + dx * pt.x - dy * pt.y;
                let y = pt_i.pt.y + dy * pt.x + dx * pt.y;
                control_points.push(Point::new(x, y));
            }

            if control_points.len() >= 2 {
                path.curve_to(control_points[0], control_points[1], pt_i1.pt);
            } else {
                path.line_to(pt_i1.pt);
            }
        }

        if self.is_closed {
            path.close_path();
        }

        path
    }

    /// Find starting index
    fn start_ix(&self) -> usize {
        if !self.is_closed {
            return 0;
        }

        for i in 0..self.ctrl_pts.len() {
            let pt = &self.ctrl_pts[i];
            if matches!(pt.ty, PointType::Corner) || pt.lth.is_some() {
                return i;
            }
        }

        // Path is all-smooth and closed
        0
    }

    /// Get point with wraparound
    fn pt(&self, i: usize, start: usize) -> &ControlPoint {
        let length = self.ctrl_pts.len();
        let index = (i + start) % length;
        &self.ctrl_pts[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{InputPoint, PointType, fit_curve};

    #[test]
    fn test_explicit_angles_horizontal() {
        // Test with explicit horizontal angles (0 degrees)
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(0.0), // horizontal
            },
            InputPoint {
                x: 100.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(0.0), // horizontal
                outgoing_angle: None,
            },
        ];

        let result = fit_curve(points, false);
        assert!(
            result.is_ok(),
            "Should successfully fit curve with explicit angles"
        );
    }

    #[test]
    fn test_explicit_angles_vertical() {
        // Test with explicit vertical angles (90 degrees)
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(90.0), // vertical up
            },
            InputPoint {
                x: 0.0,
                y: 100.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(90.0), // vertical up
                outgoing_angle: None,
            },
        ];

        let result = fit_curve(points, false);
        assert!(
            result.is_ok(),
            "Should successfully fit curve with vertical angles"
        );
    }

    #[test]
    fn test_different_angles_becomes_corner() {
        // Test that different incoming/outgoing angles create a corner
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(0.0), // horizontal
            },
            InputPoint {
                x: 100.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(0.0),  // horizontal in
                outgoing_angle: Some(90.0), // vertical out - different!
            },
            InputPoint {
                x: 100.0,
                y: 100.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
        ];

        let mut spline = Spline::new(points, false);
        spline.solve().unwrap();

        // Second point should be converted to Corner due to different angles
        assert!(
            matches!(spline.ctrl_pts[1].ty, PointType::Corner),
            "Point with different incoming/outgoing angles should become a corner"
        );
    }

    #[test]
    fn test_same_angles_stays_smooth() {
        // Test that same incoming/outgoing angles keep smooth point type
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(45.0),
            },
            InputPoint {
                x: 100.0,
                y: 100.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(45.0), // same as outgoing
                outgoing_angle: Some(45.0), // same as incoming
            },
            InputPoint {
                x: 200.0,
                y: 200.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
        ];

        let mut spline = Spline::new(points, false);
        spline.solve().unwrap();

        // Second point should remain Smooth since angles are the same
        assert!(
            matches!(spline.ctrl_pts[1].ty, PointType::Smooth),
            "Point with same incoming/outgoing angles should stay smooth"
        );
    }

    #[test]
    fn test_metapost_style_curve() {
        // Test MetaPost-style curve: z1{dir 10}..{dir 90}z2..z3
        let points = vec![
            InputPoint {
                x: 50.0,
                y: 100.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(10.0), // {dir 10}
            },
            InputPoint {
                x: 150.0,
                y: 50.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(90.0), // {dir 90}
                outgoing_angle: None,
            },
            InputPoint {
                x: 250.0,
                y: 150.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
        ];

        let result = fit_curve(points, false);
        assert!(
            result.is_ok(),
            "MetaPost-style curve should fit successfully"
        );

        let bez_path = result.unwrap();
        assert!(
            bez_path.elements().len() > 0,
            "Should generate path elements"
        );
    }

    #[test]
    fn test_no_angles_automatic() {
        // Test that curves work automatically when no angles specified
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
            InputPoint {
                x: 100.0,
                y: 100.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
            InputPoint {
                x: 200.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
        ];

        let result = fit_curve(points, false);
        assert!(result.is_ok(), "Automatic curve fitting should work");
    }

    #[test]
    fn test_degrees_to_radians_conversion() {
        // Test that degrees are properly converted to radians
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(180.0), // 180 degrees = π radians
            },
            InputPoint {
                x: 100.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: None,
            },
        ];

        let spline = Spline::new(points, false);

        // Check that the angle was converted to radians (approximately π)
        if let Some(rth) = spline.ctrl_pts[0].rth {
            let pi = std::f64::consts::PI;
            assert!(
                (rth - pi).abs() < 0.01,
                "180 degrees should be converted to approximately π radians"
            );
        }
    }
}
