use kurbo::{BezPath, PathSeg, Point, Vec2};
use std::f64::consts::PI;

use crate::spline::Spline;
use crate::two_param_curve::TwoParamCurve;
pub mod spline;
pub mod two_param_curve;
pub mod two_param_spline;
pub mod var_interpolatable_offset;
pub mod var_interpolatable_stroke;
pub mod var_interpolatable_stroker;
pub mod var_offset;
pub mod var_stroke;
pub mod var_stroker;

#[derive(Debug, Clone)]
pub struct InputPoint {
    pub x: f64,
    pub y: f64,
    pub point_type: PointType,
}

#[derive(Debug, Clone)]
pub enum PointType {
    Corner,
    Smooth,
    LineToCurve,
    CurveToLine,
}

#[derive(Debug, Clone)]
struct ControlPoint {
    pt: Point,
    ty: PointType,
    lth: Option<f64>, // left tangent angle
    rth: Option<f64>, // right tangent angle
    // Computed during solving
    l_th: Option<f64>,    // computed left tangent
    r_th: Option<f64>,    // computed right tangent
    l_ak: Option<f64>,    // left curvature
    r_ak: Option<f64>,    // right curvature
    k_blend: Option<f64>, // blended curvature
}

impl ControlPoint {
    fn new(pt: Point, ty: PointType) -> Self {
        Self {
            pt,
            ty,
            lth: None,
            rth: None,
            l_th: None,
            r_th: None,
            l_ak: None,
            r_ak: None,
            k_blend: None,
        }
    }
}

#[derive(Debug)]
pub struct CurvatureResult {
    pub ak0: f64, // arctan curvature at start
    pub ak1: f64, // arctan curvature at end
}

pub struct CurveFitter {
    curve: TwoParamCurve,
}

impl Default for CurveFitter {
    fn default() -> Self {
        Self::new()
    }
}

impl CurveFitter {
    pub fn new() -> Self {
        Self {
            curve: TwoParamCurve::new(),
        }
    }

    pub fn fit_curve(&self, points: Vec<InputPoint>, is_closed: bool) -> Result<BezPath, String> {
        let mut spline = Spline::new(points, is_closed);
        spline.solve(&self.curve)?;
        Ok(spline.render(&self.curve))
    }
}

#[derive(Debug)]
pub struct SegmentParams {
    pub th0: f64,
    pub th1: f64,
    pub chord: f64,
}

/// Normalize angle to -π..π range
fn mod2pi(th: f64) -> f64 {
    let two_pi = 2.0 * PI;
    let frac = th / two_pi;
    two_pi * (frac - frac.round())
}

// Compute endpoint tangents of a path segment.
///
/// This version is robust to the path segment not being a regular curve.
fn tangents(path: &PathSeg) -> (Vec2, Vec2) {
    const EPS: f64 = 1e-12;
    match path {
        PathSeg::Line(l) => {
            let d = l.p1 - l.p0;
            (d, d)
        }
        PathSeg::Quad(q) => {
            let d01 = q.p1 - q.p0;
            let d0 = if d01.hypot2() > EPS { d01 } else { q.p2 - q.p0 };
            let d12 = q.p2 - q.p1;
            let d1 = if d12.hypot2() > EPS { d12 } else { q.p2 - q.p0 };
            (d0, d1)
        }
        PathSeg::Cubic(c) => {
            let d01 = c.p1 - c.p0;
            let d0 = if d01.hypot2() > EPS {
                d01
            } else {
                let d02 = c.p2 - c.p0;
                if d02.hypot2() > EPS { d02 } else { c.p3 - c.p0 }
            };
            let d23 = c.p3 - c.p2;
            let d1 = if d23.hypot2() > EPS {
                d23
            } else {
                let d13 = c.p3 - c.p1;
                if d13.hypot2() > EPS { d13 } else { c.p3 - c.p0 }
            };
            (d0, d1)
        }
    }
}
