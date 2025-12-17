use kurbo::{BezPath, CubicBez, PathEl, PathSeg, Point, QuadBez, Shape};

use crate::{
    tangents,
    var_offset::offset_cubic_variable,
    var_stroke::{VariableStroke, VariableStrokeCtx},
};

pub struct VariableInterpolatableStroker {
    pub tolerance: f64,
    /// Number of binary subdivisions per cubic segment.
    /// 0 = 1 output seg per input.
    /// 1 = 2 output segs per input.
    /// 2 = 4 output segs per input.
    /// Recommended: 2 for high quality, 1 for efficiency.
    pub subdivisions: usize,
}

impl VariableInterpolatableStroker {
    pub fn new(subdivisions: usize, tolerance: f64) -> Self {
        Self {
            subdivisions,
            tolerance,
        }
    }

    /// Stroke a path with variable widths.
    /// `widths` must correspond to the on-curve points.
    pub fn stroke(
        &self,
        path: impl IntoIterator<Item = PathEl>,
        widths: &[f64],
        style: &VariableStroke,
    ) -> BezPath {
        let mut ctx = VariableStrokeCtx::default();

        // Heuristic for join threshold based on average width (safety fallback)
        // Real threshold calc happens per-join now since width changes.
        ctx.join_thresh = 2.0 * self.tolerance;

        let mut width_iter = widths.iter().cycle();

        for el in path {
            let p0 = ctx.last_pt;
            let w0 = ctx.last_width;

            match el {
                PathEl::MoveTo(p) => {
                    ctx.finish(style);
                    let w = *width_iter.next().unwrap_or(&1.0);

                    ctx.start_pt = p;
                    ctx.last_pt = p;
                    ctx.start_width = w;
                    ctx.last_width = w;
                }
                PathEl::LineTo(p1) => {
                    if p1 != p0 {
                        let w1 = *width_iter.next().unwrap_or(&w0);
                        let tangent = p1 - p0;

                        // Join at P0 using W0
                        ctx.do_join(style, tangent, w0);

                        ctx.last_tan = tangent;
                        ctx.do_line(tangent, p1, w0, w1);
                        ctx.last_width = w1;
                    }
                }
                PathEl::QuadTo(p1, p2) => {
                    if p1 != p0 || p2 != p0 {
                        let w1 = *width_iter.next().unwrap_or(&w0);
                        let q = QuadBez::new(p0, p1, p2);
                        let (tan0, tan1) = tangents(&PathSeg::Quad(q));

                        ctx.do_join(style, tan0, w0);

                        // Convert Quad to Cubic for variable offset
                        ctx.do_cubic(q.raise(), w0, w1, self.tolerance);

                        ctx.last_tan = tan1;
                        ctx.last_width = w1;
                    }
                }
                PathEl::CurveTo(p1, p2, p3) => {
                    if p1 != p0 || p2 != p0 || p3 != p0 {
                        let w1 = *width_iter.next().unwrap_or(&w0);
                        let c = CubicBez::new(p0, p1, p2, p3);
                        let (tan0, tan1) = tangents(&PathSeg::Cubic(c));

                        ctx.do_join(style, tan0, w0);

                        ctx.do_cubic(c, w0, w1, self.tolerance);

                        ctx.last_tan = tan1;
                        ctx.last_width = w1;
                    }
                }
                PathEl::ClosePath => {
                    if p0 != ctx.start_pt {
                        let w_end = ctx.start_width;
                        let tangent = ctx.start_pt - p0;

                        ctx.do_join(style, tangent, w0);
                        ctx.last_tan = tangent;
                        ctx.do_line(tangent, ctx.start_pt, w0, w_end);
                    }
                    ctx.finish_closed(style);
                }
            }
        }
        ctx.finish(style);
        ctx.output
    }
}
