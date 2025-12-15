use kurbo::{
    BezPath, CubicBez, Line, ParamCurve, PathEl, PathSeg, Point, QuadBez, Vec2,
    offset::offset_cubic,
};

/// A stroke style with variable width at path vertices.
///
/// This struct allows you to specify different stroke widths at each on-curve point
/// of a path, with smooth interpolation between them. This is useful for calligraphic
/// effects, tapered strokes, or any stroke where width varies along the path.
///
/// # Example
///
/// ```rust
/// use kurbo::BezPath;
/// use curve_fitter::variable_stroke::VariableStroke;
///
/// let mut path = BezPath::new();
/// path.move_to((0.0, 0.0));
/// path.line_to((100.0, 0.0));
/// path.line_to((100.0, 100.0));
/// path.line_to((0.0, 100.0));
/// path.close_path();
///
/// // Varying widths: thick on left, thin on top, thick on right, thin on bottom
/// let widths = vec![10.0, 5.0, 10.0, 5.0];
///
/// let stroke = VariableStroke::new(0.1);
/// let stroked = stroke.stroke(&path, &widths);
/// ```
#[derive(Clone, Debug)]
pub struct VariableStroke {
    /// The tolerance for curve approximation
    /// The tolerance parameter is passed directly to Kurbo's `offset_cubic` function
    /// and controls how closely the approximation must match the true offset curve.
    /// Higher tolerance = fewer curve subdivisions = fewer points.
    tolerance: f64,
    /// Threshold below which width variation is treated as constant
    /// (as a fraction of average width)
    /// This controls when the algorithm decides "the width variation is small enough to treat as constant."
    /// Higher values mean:
    /// - More segments treated as constant width
    /// - Less subdivision
    /// - Fewer total curve segments
    ///
    ///Recommended range: 0.15 to 0.40
    variation_threshold: f64,
    /// Maximum recursion depth for subdivision
    /// This limits how many times a curve can be subdivided for width variation.
    /// Lower values = fewer subdivisions, but may produce poor results if width varies significantly.
    /// Warning: Setting this too low can create visible artifacts when width changes are large.
    max_depth: usize,
}

impl VariableStroke {
    /// Create a new variable stroke with the given tolerance.
    ///
    /// The tolerance controls how closely the offset curves approximate
    /// the true mathematical offset.
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            variation_threshold: 0.15, // 15% variation threshold
            max_depth: 8,
        }
    }

    /// Set the variation threshold.
    ///
    /// When the width variation between two points is less than this fraction
    /// of the average width, the segment will be treated as having constant width.
    /// Default is 0.15 (15%).
    pub fn with_variation_threshold(mut self, threshold: f64) -> Self {
        self.variation_threshold = threshold;
        self
    }

    /// Set the maximum recursion depth for subdivision.
    ///
    /// Default is 8.
    pub fn with_max_depth(mut self, depth: usize) -> Self {
        self.max_depth = depth;
        self
    }

    /// Generate a stroked path with variable width.
    ///
    /// The `widths` array should contain one width value for each on-curve point
    /// in the path (excluding control points). For closed paths, the width wraps
    /// around to the first point.
    ///
    /// # Arguments
    ///
    /// * `path` - The path to stroke
    /// * `widths` - Width values at each on-curve point (half-width, radius of stroke)
    ///
    /// # Returns
    ///
    /// A new closed path representing the stroke outline.
    pub fn stroke(&self, path: &BezPath, widths: &[f64]) -> BezPath {
        if widths.is_empty() {
            return BezPath::new();
        }

        let segments: Vec<PathSeg> = path.segments().collect();
        if segments.is_empty() {
            return BezPath::new();
        }

        // Generate both sides of the stroke
        let mut left_side = BezPath::new();
        let mut right_side = BezPath::new();

        // Track current width index
        let mut width_idx = 0;

        for segment in segments.iter() {
            let w0 = widths[width_idx % widths.len()];
            let w1 = widths[(width_idx + 1) % widths.len()];

            self.stroke_segment(segment, w0, w1, &mut left_side, &mut right_side);

            width_idx += 1;
        }

        // Combine left and right sides into a closed path
        self.combine_sides(left_side, right_side)
    }

    /// Stroke a single path segment with varying width.
    fn stroke_segment(
        &self,
        segment: &PathSeg,
        w0: f64,
        w1: f64,
        left: &mut BezPath,
        right: &mut BezPath,
    ) {
        match segment {
            PathSeg::Line(line) => {
                self.stroke_line(*line, w0, w1, left, right);
            }
            PathSeg::Quad(quad) => {
                // Convert quadratic to cubic for uniform handling
                let cubic = quad.raise();
                self.stroke_cubic(cubic, w0, w1, left, right, 0);
            }
            PathSeg::Cubic(cubic) => {
                self.stroke_cubic(*cubic, w0, w1, left, right, 0);
            }
        }
    }

    /// Stroke a line segment.
    fn stroke_line(&self, line: Line, w0: f64, w1: f64, left: &mut BezPath, right: &mut BezPath) {
        let tangent = (line.p1 - line.p0).normalize();
        let normal = Vec2::new(-tangent.y, tangent.x);

        let left_p0 = line.p0 + w0 * normal;
        let left_p1 = line.p1 + w1 * normal;
        let right_p0 = line.p0 - w0 * normal;
        let right_p1 = line.p1 - w1 * normal;

        if left.is_empty() {
            left.move_to(left_p0);
        }
        left.line_to(left_p1);

        if right.is_empty() {
            right.move_to(right_p0);
        }
        right.line_to(right_p1);
    }

    /// Stroke a cubic Bézier curve with varying width.
    fn stroke_cubic(
        &self,
        cubic: CubicBez,
        w0: f64,
        w1: f64,
        left: &mut BezPath,
        right: &mut BezPath,
        depth: usize,
    ) {
        // Calculate variation as a fraction of average width
        let w_avg = (w0 + w1) / 2.0;
        let variation = if w_avg.abs() > 1e-10 {
            (w1 - w0).abs() / w_avg.abs()
        } else {
            0.0
        };

        // If variation is small or we've reached max depth, use constant width
        if variation < self.variation_threshold || depth >= self.max_depth {
            // Use average width
            self.stroke_cubic_constant(cubic, w_avg, left, right);
        } else {
            // Subdivide and recurse
            let (left_half, right_half) = cubic.subdivide();
            let w_mid = (w0 + w1) / 2.0;

            self.stroke_cubic(left_half, w0, w_mid, left, right, depth + 1);
            self.stroke_cubic(right_half, w_mid, w1, left, right, depth + 1);
        }
    }

    /// Stroke a cubic curve with constant width using Kurbo's offset algorithm.
    fn stroke_cubic_constant(
        &self,
        cubic: CubicBez,
        width: f64,
        left: &mut BezPath,
        right: &mut BezPath,
    ) {
        // Offset in positive direction (left side)
        let mut left_offset = BezPath::new();
        offset_cubic(cubic, width, self.tolerance, &mut left_offset);

        // Offset in negative direction (right side)
        let mut right_offset = BezPath::new();
        offset_cubic(cubic, -width, self.tolerance, &mut right_offset);

        // Append to accumulated paths, skipping the initial MoveTo if not first segment
        for el in left_offset.iter() {
            match el {
                PathEl::MoveTo(p) => {
                    if left.is_empty() {
                        left.move_to(p);
                    }
                }
                PathEl::LineTo(p) => left.line_to(p),
                PathEl::QuadTo(p1, p2) => left.quad_to(p1, p2),
                PathEl::CurveTo(p1, p2, p3) => left.curve_to(p1, p2, p3),
                PathEl::ClosePath => {}
            }
        }

        for el in right_offset.iter() {
            match el {
                PathEl::MoveTo(p) => {
                    if right.is_empty() {
                        right.move_to(p);
                    }
                }
                PathEl::LineTo(p) => right.line_to(p),
                PathEl::QuadTo(p1, p2) => right.quad_to(p1, p2),
                PathEl::CurveTo(p1, p2, p3) => right.curve_to(p1, p2, p3),
                PathEl::ClosePath => {}
            }
        }
    }

    /// Combine left and right sides into a closed outline.
    fn combine_sides(&self, left: BezPath, right: BezPath) -> BezPath {
        let mut result = BezPath::new();

        // Add the left side
        for el in left.iter() {
            match el {
                PathEl::MoveTo(p) => result.move_to(p),
                PathEl::LineTo(p) => result.line_to(p),
                PathEl::QuadTo(p1, p2) => result.quad_to(p1, p2),
                PathEl::CurveTo(p1, p2, p3) => result.curve_to(p1, p2, p3),
                PathEl::ClosePath => {}
            }
        }

        // Add the right side in reverse
        let right_elements: Vec<PathEl> = right.iter().collect();
        if !right_elements.is_empty() {
            // Connect to the end of right side
            if let Some(last_point) = self.get_last_point(&right_elements) {
                result.line_to(last_point);
            }

            // Traverse right side in reverse
            for el in right_elements.iter().rev() {
                match el {
                    PathEl::MoveTo(_) => {}
                    PathEl::LineTo(p) => {
                        // When going backwards through a line, we need the start point
                        // This is handled by the path structure itself
                    }
                    PathEl::QuadTo(p1, p2) => {
                        // Reverse the quadratic curve
                        result.quad_to(*p1, self.get_prev_point(&right_elements, el).unwrap());
                    }
                    PathEl::CurveTo(p1, p2, p3) => {
                        // Reverse the cubic curve
                        result.curve_to(
                            *p2,
                            *p1,
                            self.get_prev_point(&right_elements, el).unwrap(),
                        );
                    }
                    PathEl::ClosePath => {}
                }
            }
        }

        // Close the path
        result.close_path();

        result
    }

    /// Get the last point from a sequence of path elements.
    fn get_last_point(&self, elements: &[PathEl]) -> Option<Point> {
        for el in elements.iter().rev() {
            match el {
                PathEl::LineTo(p) | PathEl::MoveTo(p) => return Some(*p),
                PathEl::QuadTo(_, p) => return Some(*p),
                PathEl::CurveTo(_, _, p) => return Some(*p),
                PathEl::ClosePath => {}
            }
        }
        None
    }

    /// Get the previous point before a given element.
    fn get_prev_point(&self, elements: &[PathEl], target: &PathEl) -> Option<Point> {
        let mut current = Point::ZERO;
        for el in elements.iter() {
            if std::ptr::eq(el, target) {
                return Some(current);
            }
            match el {
                PathEl::MoveTo(p) | PathEl::LineTo(p) => current = *p,
                PathEl::QuadTo(_, p) => current = *p,
                PathEl::CurveTo(_, _, p) => current = *p,
                PathEl::ClosePath => {}
            }
        }
        None
    }
}

/// Convenience function to stroke a path with variable width.
///
/// This is a shorthand for creating a `VariableStroke` with default settings
/// and calling its `stroke` method.
pub fn stroke_variable_width(path: &BezPath, widths: &[f64], tolerance: f64) -> BezPath {
    VariableStroke::new(tolerance).stroke(path, widths)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_width() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((100.0, 100.0));
        path.line_to((0.0, 100.0));
        path.close_path();

        // All same width - should behave like constant stroke
        let widths = vec![5.0, 5.0, 5.0, 5.0];
        let stroke = VariableStroke::new(0.1);
        let result = stroke.stroke(&path, &widths);

        assert!(!result.is_empty());
    }

    #[test]
    fn test_varying_width() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((100.0, 100.0));
        path.line_to((0.0, 100.0));
        path.close_path();

        // Varying widths
        let widths = vec![10.0, 5.0, 10.0, 5.0];
        let stroke = VariableStroke::new(0.1);
        let result = stroke.stroke(&path, &widths);

        assert!(!result.is_empty());
    }

    #[test]
    fn test_single_line() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));

        let widths = vec![5.0, 10.0];
        let stroke = VariableStroke::new(0.1);
        let result = stroke.stroke(&path, &widths);

        assert!(!result.is_empty());
    }

    #[test]
    fn test_cubic_curve() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((33.0, 0.0), (67.0, 100.0), (100.0, 100.0));

        let widths = vec![5.0, 15.0];
        let stroke = VariableStroke::new(0.1);
        let result = stroke.stroke(&path, &widths);

        assert!(!result.is_empty());
    }

    #[test]
    fn test_letter_o_shape() {
        // Approximate letter 'O' with 4 cubic curves
        let mut path = BezPath::new();
        path.move_to((50.0, 0.0));
        // Top right
        path.curve_to((77.6, 0.0), (100.0, 22.4), (100.0, 50.0));
        // Bottom right
        path.curve_to((100.0, 77.6), (77.6, 100.0), (50.0, 100.0));
        // Bottom left
        path.curve_to((22.4, 100.0), (0.0, 77.6), (0.0, 50.0));
        // Top left
        path.curve_to((0.0, 22.4), (22.4, 0.0), (50.0, 0.0));
        path.close_path();

        // Thick on left/right (vertical stems), thin on top/bottom (horizontal)
        let widths = vec![
            10.0, // right
            5.0,  // bottom
            10.0, // left
            5.0,  // top
        ];

        let stroke = VariableStroke::new(0.1);
        let result = stroke.stroke(&path, &widths);

        assert!(!result.is_empty());
    }
}
