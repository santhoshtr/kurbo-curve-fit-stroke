use kurbo::{BezPath, CubicBez, PathEl, PathSeg, Point, QuadBez, Shape};

use crate::{
    tangents,
    var_offset::offset_cubic_variable,
    var_stroke::{VariableStroke, VariableStrokeCtx},
};

pub struct VariableStroker {
    pub tolerance: f64,
}

impl VariableStroker {
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_variable_stroker_new() {
        let stroker = VariableStroker::new(0.1);
        assert_eq!(stroker.tolerance, 0.1);
    }

    #[test]
    fn test_stroke_empty_path() {
        let path = BezPath::new();
        let style = VariableStroke::default();
        let widths = vec![5.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);
        assert_eq!(result.elements().len(), 0);
    }

    #[test]
    fn test_stroke_simple_line_constant_width() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));

        let style = VariableStroke::default();
        let widths = vec![10.0, 10.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a closed path
        assert!(result.elements().len() > 0);

        // Check that the path is closed
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_simple_line_varying_width() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));

        let widths = vec![5.0, 15.0];
        let style = VariableStroke::default();
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a closed path
        assert!(result.elements().len() > 0);

        // Check that the path is closed
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_square_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((100.0, 100.0));
        path.line_to((0.0, 100.0));
        path.close_path();

        let style = VariableStroke::default();
        let widths = vec![10.0, 5.0, 10.0, 5.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a closed path with elements
        assert!(result.elements().len() > 4);

        // Verify the result has a bounding box
        let bbox = result.bounding_box();
        assert!(bbox.width() > 0.0);
        assert!(bbox.height() > 0.0);
    }

    #[test]
    fn test_stroke_cubic_bezier() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((33.0, 33.0), (66.0, 33.0), (100.0, 0.0));

        let style = VariableStroke::default();
        let widths = vec![10.0, 15.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a closed path
        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_quadratic_bezier() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.quad_to((50.0, 50.0), (100.0, 0.0));

        let style = VariableStroke::default();
        let widths = vec![8.0, 12.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a closed path
        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_width_wrapping_closed_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((50.0, 0.0));
        path.line_to((50.0, 50.0));
        path.close_path();

        let style = VariableStroke::default();
        // Only provide 2 widths for 3 segments - should wrap around
        let widths = vec![10.0, 5.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should still produce a valid closed path
        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_s_curve() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((50.0, 100.0), (50.0, -100.0), (100.0, 0.0));

        let style = VariableStroke::default();
        let widths = vec![8.0, 12.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_with_different_tolerances() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((0.0, 50.0), (100.0, 50.0), (100.0, 0.0));

        let widths = vec![10.0, 10.0];

        let style = VariableStroke::default();
        // Test with different tolerances
        let stroker_low_tol = VariableStroker::new(0.01);
        let result_low_tol = stroker_low_tol.stroke(&path, &widths, &style);

        let stroker_high_tol = VariableStroker::new(1.0);
        let result_high_tol = stroker_high_tol.stroke(&path, &widths, &style);

        // Both should produce valid paths
        assert!(result_low_tol.elements().len() > 0);
        assert!(result_high_tol.elements().len() > 0);
    }

    #[test]
    fn test_stroke_large_width_variation() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((33.3, 33.3), (66.6, 33.3), (100.0, 0.0));

        let style = VariableStroke::default();
        let widths = vec![5.0, 50.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_stroke_multi_segment_open_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((50.0, 0.0));
        path.line_to((50.0, 50.0));
        path.line_to((100.0, 50.0));

        let style = VariableStroke::default();
        let widths = vec![10.0, 8.0, 12.0, 6.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a closed path
        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }

    #[test]
    fn test_stroke_mixed_segment_types() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((50.0, 0.0));
        path.quad_to((75.0, 25.0), (100.0, 0.0));
        path.curve_to((100.0, 50.0), (50.0, 50.0), (0.0, 50.0));

        let style = VariableStroke::default();
        let widths = vec![10.0, 8.0, 12.0, 6.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths, &style);

        // Should produce a valid closed path
        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }
}
