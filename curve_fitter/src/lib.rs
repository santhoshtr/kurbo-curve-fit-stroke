use kurbo::{BezPath, Point};
use std::f64::consts::PI;

use crate::spline::Spline;
use crate::two_param_curve::TwoParamCurve;
pub mod spline;
pub mod two_param_curve;
pub mod two_param_spline;

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
