use kurbo::{BezPath, CubicBez, PathEl, PathSeg, Point, QuadBez};

use crate::{
    tangents,
    var_stroke::{VariableStroke, VariableStrokeCtx},
};

/// Number of widths `stroke` expects: one per on-curve point. Degenerate
/// segments contribute no point, and the closing segment of a subpath that
/// returns to its start reuses the start width rather than expecting its own.
fn count_width_points(elements: &[PathEl]) -> usize {
    let mut count = 0;
    let mut start = Point::ZERO;
    let mut pos = Point::ZERO;
    // Whether the last width-consuming segment of the current subpath ended
    // back at the subpath start (its width wraps to the start width)
    let mut wrapped = false;

    for el in elements {
        match *el {
            PathEl::MoveTo(p) => {
                if wrapped {
                    count -= 1;
                }
                wrapped = false;
                count += 1;
                start = p;
                pos = p;
            }
            PathEl::LineTo(p1) => {
                if p1 != pos {
                    count += 1;
                    pos = p1;
                    wrapped = pos == start;
                }
            }
            PathEl::QuadTo(p1, p2) => {
                if p1 != pos || p2 != pos {
                    count += 1;
                    pos = p2;
                    wrapped = pos == start;
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != pos || p2 != pos || p3 != pos {
                    count += 1;
                    pos = p3;
                    wrapped = pos == start;
                }
            }
            PathEl::ClosePath => {
                pos = start;
            }
        }
    }
    if wrapped {
        count -= 1;
    }

    count
}

pub struct VariableStroker {
    pub tolerance: f64,
}

impl VariableStroker {
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }

    /// Stroke a path with variable widths.
    ///
    /// `widths` must contain exactly one width per on-curve point of the
    /// path; a closed subpath's seam reuses the first width. Returns an
    /// error if the count does not match.
    pub fn stroke(
        &self,
        path: impl IntoIterator<Item = PathEl>,
        widths: &[f64],
        style: &VariableStroke,
    ) -> Result<BezPath, String> {
        let elements: Vec<PathEl> = path.into_iter().collect();

        let expected = count_width_points(&elements);
        if widths.len() != expected {
            return Err(format!(
                "Width count mismatch: path has {} on-curve points, got {} widths",
                expected,
                widths.len()
            ));
        }

        let mut ctx = VariableStrokeCtx::default();

        // Heuristic for join threshold based on average width (safety fallback)
        // Real threshold calc happens per-join now since width changes.
        ctx.join_thresh = 2.0 * self.tolerance;

        // The cycle supplies the wrap width at the seam of closed subpaths
        let mut width_iter = widths.iter().cycle();

        for el in elements {
            let p0 = ctx.last_pt;
            let w0 = ctx.last_width;

            match el {
                PathEl::MoveTo(p) => {
                    ctx.finish(style, self.tolerance);
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
                        ctx.do_join(style, tangent, w0, self.tolerance);

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

                        ctx.do_join(style, tan0, w0, self.tolerance);

                        // Convert Quad to Cubic for variable offset
                        ctx.do_cubic(q.raise(), w0, w1);

                        ctx.last_tan = tan1;
                        ctx.last_width = w1;
                    }
                }
                PathEl::CurveTo(p1, p2, p3) => {
                    if p1 != p0 || p2 != p0 || p3 != p0 {
                        let w1 = *width_iter.next().unwrap_or(&w0);
                        let c = CubicBez::new(p0, p1, p2, p3);
                        let (tan0, tan1) = tangents(&PathSeg::Cubic(c));

                        ctx.do_join(style, tan0, w0, self.tolerance);

                        ctx.do_cubic(c, w0, w1);

                        ctx.last_tan = tan1;
                        ctx.last_width = w1;
                    }
                }
                PathEl::ClosePath => {
                    if p0 != ctx.start_pt {
                        let w_end = ctx.start_width;
                        let tangent = ctx.start_pt - p0;

                        ctx.do_join(style, tangent, w0, self.tolerance);
                        ctx.last_tan = tangent;
                        ctx.do_line(tangent, ctx.start_pt, w0, w_end);
                    }
                    ctx.finish_closed(style, self.tolerance);
                }
            }
        }
        ctx.finish(style, self.tolerance);
        Ok(ctx.output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_width_count_open_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.curve_to((120.0, 20.0), (140.0, 40.0), (160.0, 40.0));

        // 3 on-curve points
        assert_eq!(count_width_points(path.elements()), 3);

        let stroker = VariableStroker::new(0.1);
        let style = VariableStroke::default();
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0], &style).is_ok());
        assert!(stroker.stroke(&path, &[10.0, 12.0], &style).is_err());
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0, 9.0], &style).is_err());
    }

    #[test]
    fn test_width_count_closed_path() {
        // Closed path whose final segment returns to the start: the seam
        // reuses the first width, so 3 distinct points need 3 widths
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((50.0, 80.0));
        path.line_to((0.0, 0.0));
        path.close_path();

        assert_eq!(count_width_points(path.elements()), 3);

        let stroker = VariableStroker::new(0.1);
        let style = VariableStroke::default();
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0], &style).is_ok());
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0, 10.0], &style).is_err());
    }
}
