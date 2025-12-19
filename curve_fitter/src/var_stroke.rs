use std::f64::consts::PI;

use kurbo::{Affine, Arc, BezPath, Cap, CubicBez, Join, PathEl, Point, Vec2};

use crate::var_offset::offset_cubic_variable;

#[derive(Clone, Debug, PartialEq)]
pub struct VariableStroke {
    pub join: Join,
    pub miter_limit: f64,
    pub start_cap: Cap,
    pub end_cap: Cap,
}

impl Default for VariableStroke {
    fn default() -> Self {
        Self {
            join: Join::Round,
            miter_limit: 4.0,
            start_cap: Cap::Round,
            end_cap: Cap::Round,
        }
    }
}

impl VariableStroke {
    /// Builder method for setting the join style.
    pub fn with_join(mut self, join: Join) -> Self {
        self.join = join;
        self
    }

    /// Builder method for setting the limit for miter joins.
    pub fn with_miter_limit(mut self, limit: f64) -> Self {
        self.miter_limit = limit;
        self
    }

    /// Builder method for setting the cap style for the start of the stroke.
    pub fn with_start_cap(mut self, cap: Cap) -> Self {
        self.start_cap = cap;
        self
    }

    /// Builder method for setting the cap style for the end of the stroke.
    pub fn with_end_cap(mut self, cap: Cap) -> Self {
        self.end_cap = cap;
        self
    }

    /// Builder method for setting the cap style.
    pub fn with_caps(mut self, cap: Cap) -> Self {
        self.start_cap = cap;
        self.end_cap = cap;
        self
    }
}

// We reuse the StrokeCtx almost exactly, but we need to track the start_width
// to handle ClosePath correctly.
#[derive(Default, Debug)]
pub struct VariableStrokeCtx {
    pub(crate) output: BezPath,
    forward_path: BezPath,
    backward_path: BezPath,
    result_path: BezPath, // Scratch space

    pub(crate) start_pt: Point,
    start_norm: Vec2,
    start_tan: Vec2,
    pub(crate) start_width: f64, // <--- NEW: needed for closing the loop

    pub(crate) last_pt: Point,
    pub(crate) last_tan: Vec2,
    pub(crate) last_width: f64, // <--- NEW: width at the current join

    pub join_thresh: f64,
}

impl VariableStrokeCtx {
    fn reset(&mut self) {
        self.output.truncate(0);
        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
        self.start_pt = Point::default();
        self.start_norm = Vec2::default();
        self.start_tan = Vec2::default();
        self.start_width = 0.0;
        self.last_pt = Point::default();
        self.last_tan = Vec2::default();
        self.last_width = 0.0;
        self.join_thresh = 0.0;
    }

    pub fn output(&self) -> &BezPath {
        &self.output
    }

    // Modified to accept `width` for the specific join
    pub fn do_join(&mut self, style: &VariableStroke, tan0: Vec2, width: f64, tolerance: f64) {
        // Calculate join threshold dynamically for this width
        let join_thresh = if width > 1e-6 {
            2.0 * tolerance / width
        } else {
            0.0
        };

        // Offset normal at the start of the new segment
        let scale = 0.5 * width / tan0.hypot();
        let norm = scale * Vec2::new(-tan0.y, tan0.x);
        let p0 = self.last_pt;

        if self.forward_path.elements().is_empty() {
            // First point of the path
            self.forward_path.move_to(p0 - norm);
            self.backward_path.move_to(p0 + norm);
            self.start_tan = tan0;
            self.start_norm = norm;
        } else {
            // Stitching logic
            let ab = self.last_tan;
            let cd = tan0;
            let cross = ab.cross(cd);
            let dot = ab.dot(cd);
            let hypot = cross.hypot(dot);

            if dot <= 0.0 || cross.abs() >= hypot * join_thresh {
                match style.join {
                    Join::Bevel => {
                        self.forward_path.line_to(p0 - norm);
                        self.backward_path.line_to(p0 + norm);
                    }
                    Join::Miter => {
                        if 2.0 * hypot < (hypot + dot) * style.miter_limit.powi(2) {
                            // Calculate last_norm based on last_width
                            let last_scale = 0.5 * self.last_width / ab.hypot();
                            let last_norm = last_scale * Vec2::new(-ab.y, ab.x);

                            if cross > 0.0 {
                                // Outer miter on Forward path
                                let fp_last = p0 - last_norm;
                                let fp_this = p0 - norm;
                                let h = ab.cross(fp_this - fp_last) / cross;
                                let miter_pt = fp_this - cd * h;
                                self.forward_path.line_to(miter_pt);
                                self.backward_path.line_to(p0); // Inner point
                            } else {
                                // Outer miter on Backward path
                                let fp_last = p0 + last_norm;
                                let fp_this = p0 + norm;
                                let h = ab.cross(fp_this - fp_last) / cross;
                                let miter_pt = fp_this - cd * h;
                                self.backward_path.line_to(miter_pt);
                                self.forward_path.line_to(p0); // Inner point
                            }
                        } else {
                            // Fallback to Bevel
                            self.forward_path.line_to(p0 - norm);
                            self.backward_path.line_to(p0 + norm);
                        }
                    }
                    Join::Round => {
                        let angle = cross.atan2(dot);
                        if angle > 0.0 {
                            self.backward_path.line_to(p0 + norm);
                            round_join(&mut self.forward_path, tolerance, p0, norm, angle);
                        } else {
                            self.forward_path.line_to(p0 - norm);
                            // Note: We use -norm for the reverse path
                            round_join_rev(&mut self.backward_path, tolerance, p0, -norm, -angle);
                        }
                    }
                }
            } else {
                // Tangents are aligned (or width is tiny), just connect
                self.forward_path.line_to(p0 - norm);
                self.backward_path.line_to(p0 + norm);
            }
        }
    }

    pub fn do_line(&mut self, tangent: Vec2, p1: Point, _w0: f64, w1: f64) {
        // Calculate norm at the end (using w1)
        // The start of this segment was already handled by do_join (using w0)
        let scale = 0.5 * w1 / tangent.hypot();
        let norm = scale * Vec2::new(-tangent.y, tangent.x);

        // Forward path (Right hand side usually, here -norm)
        self.forward_path.line_to(p1 - norm);
        // Backward path (Left hand side usually, here +norm)
        self.backward_path.line_to(p1 + norm);

        self.last_pt = p1;
    }

    pub fn do_cubic(&mut self, c: CubicBez, w0: f64, w1: f64, tolerance: f64) {
        // 1. Forward Path (Right Side in Kurbo convention -> negative width offset)
        // We use -0.5 * width.
        // offset_cubic_variable must be imported
        offset_cubic_variable(c, -0.5 * w0, -0.5 * w1, tolerance, &mut self.result_path);

        // The first point of result_path is the "MoveTo" which corresponds to the
        // end of the join. We usually want to connect to it.
        // However, Kurbo usually skips the first point assuming it matches.
        // In variable width, exact matches are guaranteed by the math.
        self.forward_path.extend(self.result_path.iter().skip(1));

        // 2. Backward Path (Left Side -> positive width offset)
        offset_cubic_variable(c, 0.5 * w0, 0.5 * w1, tolerance, &mut self.result_path);
        self.backward_path.extend(self.result_path.iter().skip(1));

        self.last_pt = c.p3;
    }

    // Reuse finish() and finish_closed() from original code
    // They work perfectly because they operate on the accumulated forward/backward paths.
    pub fn finish(&mut self, style: &VariableStroke, tolerance: f64) {
        if self.forward_path.is_empty() {
            return;
        }

        self.output.extend(&self.forward_path);

        let back_els = self.backward_path.elements();
        let return_p = back_els.last().unwrap().end_point().unwrap();

        // End Cap (at last_pt)
        let d = self.last_pt - return_p;
        // Note: d vector here represents the full width vector at the end.

        match style.end_cap {
            Cap::Butt => self.output.line_to(return_p),
            Cap::Round => round_cap(&mut self.output, tolerance, self.last_pt, d),
            Cap::Square => square_cap(&mut self.output, false, self.last_pt, d),
        }

        extend_reversed(&mut self.output, back_els);

        // Start Cap (at start_pt)
        match style.start_cap {
            Cap::Butt => self.output.close_path(),
            Cap::Round => round_cap(&mut self.output, tolerance, self.start_pt, self.start_norm),
            Cap::Square => square_cap(&mut self.output, true, self.start_pt, self.start_norm),
        }

        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
    }

    pub fn finish_closed(&mut self, style: &VariableStroke, tolerance: f64) {
        if self.forward_path.is_empty() {
            return;
        }

        // Join the end to the start
        self.do_join(style, self.start_tan, self.start_width, tolerance);

        self.output.extend(&self.forward_path);
        self.output.close_path();

        let back_els = self.backward_path.elements();
        let last_pt = back_els.last().unwrap().end_point().unwrap();

        self.output.move_to(last_pt);
        extend_reversed(&mut self.output, back_els);
        self.output.close_path();

        self.forward_path.truncate(0);
        self.backward_path.truncate(0);
    }
}

// Helpers for Caps (Keep exactly the same from StrokeCtx)
fn round_cap(out: &mut BezPath, tolerance: f64, center: Point, norm: Vec2) {
    round_join(out, tolerance, center, norm, PI);
}

fn round_join(out: &mut BezPath, tolerance: f64, center: Point, norm: Vec2, angle: f64) {
    let a = Affine::new([norm.x, norm.y, -norm.y, norm.x, center.x, center.y]);
    let arc = Arc::new(Point::ORIGIN, (1.0, 1.0), PI - angle, angle, 0.0);
    arc.to_cubic_beziers(tolerance, |p1, p2, p3| out.curve_to(a * p1, a * p2, a * p3));
}

fn round_join_rev(out: &mut BezPath, tolerance: f64, center: Point, norm: Vec2, angle: f64) {
    let a = Affine::new([norm.x, norm.y, norm.y, -norm.x, center.x, center.y]);
    let arc = Arc::new(Point::ORIGIN, (1.0, 1.0), PI - angle, angle, 0.0);
    arc.to_cubic_beziers(tolerance, |p1, p2, p3| out.curve_to(a * p1, a * p2, a * p3));
}

fn square_cap(out: &mut BezPath, close: bool, center: Point, norm: Vec2) {
    let a = Affine::new([norm.x, norm.y, -norm.y, norm.x, center.x, center.y]);
    out.line_to(a * Point::new(1.0, 1.0));
    out.line_to(a * Point::new(-1.0, 1.0));
    if close {
        out.close_path();
    } else {
        out.line_to(a * Point::new(-1.0, 0.0));
    }
}

fn extend_reversed(out: &mut BezPath, elements: &[PathEl]) {
    for i in (1..elements.len()).rev() {
        let end = elements[i - 1].end_point().unwrap();
        match elements[i] {
            PathEl::LineTo(_) => out.line_to(end),
            PathEl::QuadTo(p1, _) => out.quad_to(p1, end),
            PathEl::CurveTo(p1, p2, _) => out.curve_to(p2, p1, end),
            _ => {}
        }
    }
}
