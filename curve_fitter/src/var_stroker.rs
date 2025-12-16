use kurbo::{BezPath, CubicBez, ParamCurveArclen, PathEl, PathSeg, Point, Shape, Vec2};

use crate::{tangents, var_offset::offset_cubic_variable};

pub struct VariableStroker {
    pub tolerance: f64,
}

impl VariableStroker {
    pub fn new(tolerance: f64) -> Self {
        Self { tolerance }
    }

    /// Stroke a BezPath with variable widths.
    ///
    /// * `path`: The source geometry.
    /// * `widths`: An array of widths corresponding to the path's anchor points.
    ///             Must have at least `segments.count() + 1` for open paths,
    ///             or `segments.count()` for closed paths.
    pub fn stroke(&self, path: &BezPath, widths: &[f64]) -> BezPath {
        let segments: Vec<PathSeg> = path.segments().collect();
        if segments.is_empty() {
            return BezPath::new();
        }

        let count = segments.len();
        let is_closed = path
            .elements()
            .iter()
            .any(|el| matches!(el, PathEl::ClosePath));

        // Step 1: Calculate ideal offset tangents at each node
        let (left_tangents, right_tangents) =
            self.calculate_offset_tangents(&segments, widths, is_closed, count);

        // Step 2: Generate offset curves for left and right sides
        let (left_chains, right_chains) =
            self.generate_offset_chains(&segments, widths, &left_tangents, &right_tangents, count);

        // Step 3: Stitch the offset chains into a closed path
        self.stitch_offset_chains(left_chains, right_chains, is_closed)
    }

    /// Calculate ideal offset tangents for smooth joins.
    ///
    /// For each node where the source curve is G1 continuous (smooth), we calculate
    /// blended tangent vectors for both the left and right offset curves. This ensures
    /// the offset maintains smoothness at joins.
    ///
    /// Returns a tuple of (left_tangents, right_tangents), each containing Option<Vec2>
    /// for each node position.
    fn calculate_offset_tangents(
        &self,
        segments: &[PathSeg],
        widths: &[f64],
        is_closed: bool,
        count: usize,
    ) -> (Vec<Option<Vec2>>, Vec<Option<Vec2>>) {
        let mut left_tangents = vec![None; count + 1];
        let mut right_tangents = vec![None; count + 1];

        for i in 0..=count {
            // Skip endpoints for open paths
            if !is_closed && (i == 0 || i == count) {
                continue;
            }

            // Calculate indices for previous and current segments
            let prev_idx = if i == 0 { count - 1 } else { i - 1 };
            let curr_idx = if i == count { 0 } else { i };

            // Bounds check for open paths
            if !is_closed && (prev_idx >= count || curr_idx >= count) {
                continue;
            }

            let prev_seg = segments[prev_idx];
            let curr_seg = segments[curr_idx];

            // Get source tangents at this node
            let (_start_tangent, tan_in) = tangents(&prev_seg);
            let (tan_out, _end_tangent) = tangents(&curr_seg);

            // Check if source is G1 continuous (smooth join)
            // Dot product close to 1.0 means vectors are aligned
            let is_source_smooth = tan_in.dot(tan_out) > 0.99;

            if is_source_smooth {
                // Calculate width derivatives (rate of change per unit arc length)
                let w_prev_start = widths[prev_idx % widths.len()];
                let w_node = widths[i % widths.len()];
                let w_next_end = widths[(i + 1) % widths.len()];

                let len_prev = prev_seg.arclen(1e-3);
                let len_curr = curr_seg.arclen(1e-3);

                // Width slopes: w' = dw/ds
                let w_prime_in = (w_node - w_prev_start) / len_prev;
                let w_prime_out = (w_next_end - w_node) / len_curr;

                // Calculate left side tangent
                // Offset tangent: T_offset = T + w' * N
                // where T is the source tangent and N is the normal
                let n_in = Vec2::new(-tan_in.y, tan_in.x);
                let n_out = Vec2::new(-tan_out.y, tan_out.x);

                let t_off_in_left = tan_in + n_in * w_prime_in;
                let t_off_out_left = tan_out + n_out * w_prime_out;

                // Average and normalize to get the blended tangent
                let avg_left = (t_off_in_left.normalize() + t_off_out_left.normalize()).normalize();
                left_tangents[i] = Some(avg_left);

                // Calculate right side tangent
                // Right side uses negative width, so slope is -w'
                let t_off_in_right = tan_in - n_in * w_prime_in;
                let t_off_out_right = tan_out - n_out * w_prime_out;

                let avg_right =
                    (t_off_in_right.normalize() + t_off_out_right.normalize()).normalize();
                right_tangents[i] = Some(avg_right);
            }
        }

        // Handle wrap-around for closed paths
        if is_closed {
            left_tangents[0] = left_tangents[count];
            right_tangents[0] = right_tangents[count];
        }

        (left_tangents, right_tangents)
    }

    /// Generate offset curve chains for both left and right sides.
    ///
    /// For each segment in the source path, generate offset curves with variable width.
    /// The offset curves use the pre-calculated locked tangents at smooth joins.
    ///
    /// Returns a tuple of (left_chains, right_chains), where each chain is a Vec<PathEl>
    /// representing the offset result of a single source segment.
    fn generate_offset_chains(
        &self,
        segments: &[PathSeg],
        widths: &[f64],
        left_tangents: &[Option<Vec2>],
        right_tangents: &[Option<Vec2>],
        count: usize,
    ) -> (Vec<Vec<PathEl>>, Vec<Vec<PathEl>>) {
        let mut left_chains: Vec<Vec<PathEl>> = Vec::with_capacity(count);
        let mut right_chains: Vec<Vec<PathEl>> = Vec::with_capacity(count);

        for i in 0..count {
            let seg = segments[i];

            // Get widths for this segment's start and end points
            let w0 = widths[i % widths.len()];
            let w1 = widths[(i + 1) % widths.len()];

            // Get locked tangents if they exist
            let l_tan0: Option<Vec2> = left_tangents[i];
            let l_tan1: Option<Vec2> = left_tangents[i + 1];
            let r_tan0: Option<Vec2> = right_tangents[i];
            let r_tan1: Option<Vec2> = right_tangents[i + 1];

            // Convert segment to cubic bezier for uniform processing
            let cubic: CubicBez = self.segment_to_cubic(seg);

            // Generate left side offset (positive width)
            let mut l_path = BezPath::new();
            offset_cubic_variable(cubic, w0, w1, l_tan0, l_tan1, self.tolerance, &mut l_path);
            left_chains.push(l_path.elements().to_vec());

            // Generate right side offset (negative width)
            let mut r_path = BezPath::new();
            offset_cubic_variable(cubic, -w0, -w1, r_tan0, r_tan1, self.tolerance, &mut r_path);
            right_chains.push(r_path.elements().to_vec());
        }

        (left_chains, right_chains)
    }

    /// Convert any path segment to a cubic bezier.
    ///
    /// Lines are converted explicitly to preserve t-parameter correspondence.
    /// Quadratic beziers are degree-elevated to cubic.
    fn segment_to_cubic(&self, seg: PathSeg) -> CubicBez {
        match seg {
            PathSeg::Line(l) => {
                // Convert line to cubic explicitly to preserve t-values
                CubicBez::new(
                    l.p0,
                    l.p0 + (l.p1 - l.p0) * (1.0 / 3.0),
                    l.p1 - (l.p1 - l.p0) * (1.0 / 3.0),
                    l.p1,
                )
            }
            PathSeg::Quad(q) => q.raise(),
            PathSeg::Cubic(c) => c,
        }
    }

    /// Stitch offset chains into a complete closed path.
    ///
    /// Combines left side (forward), tip cap, right side (reversed), and base cap
    /// into a single closed outline. Joins between segments use bevel joins (straight lines)
    /// for robustness at sharp corners.
    fn stitch_offset_chains(
        &self,
        left_chains: Vec<Vec<PathEl>>,
        right_chains: Vec<Vec<PathEl>>,
        is_closed: bool,
    ) -> BezPath {
        let mut result = BezPath::new();

        // Stitch left side (forward direction)
        self.append_side_forward(&mut result, &left_chains, is_closed);

        // Add tip cap (for open paths)
        if !is_closed {
            self.append_tip_cap(&mut result, &right_chains);
        } else {
            // For closed paths, jump to the end of the right side's last segment
            if let Some(last_chain) = right_chains.last() {
                if let Some(end_point) = get_end_point(last_chain) {
                    result.line_to(end_point);
                }
            }
        }

        // Stitch right side (reverse direction)
        self.append_side_reversed(&mut result, &right_chains, is_closed);

        // Close the path
        result.close_path();
        result
    }

    /// Append the left side offset chains in forward direction.
    ///
    /// Chains are connected with bevel joins (straight line segments) between
    /// the end of one chain and the start of the next.
    fn append_side_forward(&self, result: &mut BezPath, chains: &[Vec<PathEl>], is_closed: bool) {
        for (i, chain) in chains.iter().enumerate() {
            for (j, el) in chain.iter().enumerate() {
                match el {
                    PathEl::MoveTo(p) => {
                        // Only the very first point needs a MoveTo
                        if i == 0 && j == 0 {
                            result.move_to(*p);
                        }
                    }
                    PathEl::LineTo(p) => result.line_to(*p),
                    PathEl::QuadTo(p1, p2) => result.quad_to(*p1, *p2),
                    PathEl::CurveTo(p1, p2, p3) => result.curve_to(*p1, *p2, *p3),
                    PathEl::ClosePath => {}
                }
            }

            // Add bevel join to next segment
            if i < chains.len() - 1 || is_closed {
                let next_idx = (i + 1) % chains.len();
                let next_start = get_start_point(&chains[next_idx]);
                result.line_to(next_start);
            }
        }
    }

    /// Append the tip cap connecting left side to right side.
    ///
    /// For open paths, connects the end of the left side to the end of the right side.
    fn append_tip_cap(&self, result: &mut BezPath, right_chains: &[Vec<PathEl>]) {
        if let Some(last_chain) = right_chains.last() {
            if let Some(end_point) = get_end_point(last_chain) {
                result.line_to(end_point);
            }
        }
    }

    /// Append the right side offset chains in reverse direction.
    ///
    /// The right side is traversed backwards (last segment to first) and each
    /// segment's geometry is reversed to maintain proper orientation.
    fn append_side_reversed(&self, result: &mut BezPath, chains: &[Vec<PathEl>], is_closed: bool) {
        for i in (0..chains.len()).rev() {
            let chain = &chains[i];

            // Convert chain to segments and reverse them
            let chain_iter = ChainIterator::new(chain);
            let segments: Vec<PathSeg> = chain_iter.collect();

            for seg in segments.into_iter().rev() {
                let rev_seg = seg.reverse();
                match rev_seg {
                    PathSeg::Line(l) => result.line_to(l.p1),
                    PathSeg::Quad(q) => result.quad_to(q.p1, q.p2),
                    PathSeg::Cubic(c) => result.curve_to(c.p1, c.p2, c.p3),
                }
            }

            // Add bevel join to the next segment (in reverse order)
            if i > 0 || is_closed {
                let next_idx = if i == 0 { chains.len() - 1 } else { i - 1 };
                let next_end = get_end_point(&chains[next_idx]).unwrap();
                result.line_to(next_end);
            }
        }
    }
}

/// Get the starting point of a path element chain.
fn get_start_point(chain: &[PathEl]) -> Point {
    match chain.first() {
        Some(PathEl::MoveTo(p)) => *p,
        _ => Point::ZERO,
    }
}

/// Get the ending point of a path element chain.
fn get_end_point(chain: &[PathEl]) -> Option<Point> {
    match chain.last() {
        Some(PathEl::MoveTo(p)) | Some(PathEl::LineTo(p)) => Some(*p),
        Some(PathEl::QuadTo(_, p)) => Some(*p),
        Some(PathEl::CurveTo(_, _, p)) => Some(*p),
        _ => None,
    }
}

/// Iterator that converts a slice of PathEl into PathSeg segments.
struct ChainIterator<'a> {
    elements: &'a [PathEl],
    index: usize,
    last_point: Point,
}

impl<'a> ChainIterator<'a> {
    fn new(elements: &'a [PathEl]) -> Self {
        let start = match elements.first() {
            Some(PathEl::MoveTo(p)) => *p,
            _ => Point::ZERO,
        };
        Self {
            elements,
            index: 0,
            last_point: start,
        }
    }
}

impl<'a> Iterator for ChainIterator<'a> {
    type Item = PathSeg;

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.elements.len() {
            let el = &self.elements[self.index];
            self.index += 1;
            match el {
                PathEl::MoveTo(p) => {
                    self.last_point = *p;
                    continue;
                }
                PathEl::LineTo(p) => {
                    let seg = PathSeg::Line(kurbo::Line::new(self.last_point, *p));
                    self.last_point = *p;
                    return Some(seg);
                }
                PathEl::QuadTo(p1, p2) => {
                    let seg = PathSeg::Quad(kurbo::QuadBez::new(self.last_point, *p1, *p2));
                    self.last_point = *p2;
                    return Some(seg);
                }
                PathEl::CurveTo(p1, p2, p3) => {
                    let seg = PathSeg::Cubic(CubicBez::new(self.last_point, *p1, *p2, *p3));
                    self.last_point = *p3;
                    return Some(seg);
                }
                PathEl::ClosePath => continue,
            }
        }
        None
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
        let widths = vec![5.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);
        assert_eq!(result.elements().len(), 0);
    }

    #[test]
    fn test_stroke_simple_line_constant_width() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));

        let widths = vec![10.0, 10.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        let widths = vec![10.0, 5.0, 10.0, 5.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        let widths = vec![10.0, 15.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        let widths = vec![8.0, 12.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        // Only provide 2 widths for 3 segments - should wrap around
        let widths = vec![10.0, 5.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        let widths = vec![8.0, 12.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        // Test with different tolerances
        let stroker_low_tol = VariableStroker::new(0.01);
        let result_low_tol = stroker_low_tol.stroke(&path, &widths);

        let stroker_high_tol = VariableStroker::new(1.0);
        let result_high_tol = stroker_high_tol.stroke(&path, &widths);

        // Both should produce valid paths
        assert!(result_low_tol.elements().len() > 0);
        assert!(result_high_tol.elements().len() > 0);
    }

    #[test]
    fn test_get_start_point() {
        let elements = vec![
            PathEl::MoveTo(Point::new(10.0, 20.0)),
            PathEl::LineTo(Point::new(30.0, 40.0)),
        ];

        let start = get_start_point(&elements);
        assert_eq!(start, Point::new(10.0, 20.0));
    }

    #[test]
    fn test_get_end_point() {
        let elements = vec![
            PathEl::MoveTo(Point::new(10.0, 20.0)),
            PathEl::LineTo(Point::new(30.0, 40.0)),
        ];

        let end = get_end_point(&elements);
        assert_eq!(end, Some(Point::new(30.0, 40.0)));
    }

    #[test]
    fn test_get_end_point_cubic() {
        let elements = vec![
            PathEl::MoveTo(Point::new(0.0, 0.0)),
            PathEl::CurveTo(
                Point::new(10.0, 10.0),
                Point::new(20.0, 20.0),
                Point::new(30.0, 0.0),
            ),
        ];

        let end = get_end_point(&elements);
        assert_eq!(end, Some(Point::new(30.0, 0.0)));
    }

    #[test]
    fn test_chain_iterator() {
        let elements = vec![
            PathEl::MoveTo(Point::new(0.0, 0.0)),
            PathEl::LineTo(Point::new(10.0, 0.0)),
            PathEl::LineTo(Point::new(10.0, 10.0)),
        ];

        let chain_iter = ChainIterator::new(&elements);
        let segments: Vec<PathSeg> = chain_iter.collect();

        assert_eq!(segments.len(), 2);
    }

    #[test]
    fn test_stroke_large_width_variation() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((33.3, 33.3), (66.6, 33.3), (100.0, 0.0));

        let widths = vec![5.0, 50.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

        assert!(result.elements().len() > 0);
    }

    #[test]
    fn test_stroke_multi_segment_open_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((50.0, 0.0));
        path.line_to((50.0, 50.0));
        path.line_to((100.0, 50.0));

        let widths = vec![10.0, 8.0, 12.0, 6.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

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

        let widths = vec![10.0, 8.0, 12.0, 6.0];
        let stroker = VariableStroker::new(0.1);
        let result = stroker.stroke(&path, &widths);

        // Should produce a valid closed path
        assert!(result.elements().len() > 0);
        let last_element = result.elements().last();
        assert!(matches!(last_element, Some(PathEl::ClosePath)));
    }
}
