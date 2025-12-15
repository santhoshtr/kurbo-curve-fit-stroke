use kurbo::{BezPath, CubicBez, PathEl, PathSeg, Point, Shape};

use crate::var_offset::offset_cubic_variable;

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

        // We accumulate the generated segments for left and right sides separateley
        // so we can reverse the right side later.
        // Inner Vec<PathEl> represents the offset result of a SINGLE source segment
        // (which might be subdivided into multiple curves).
        let mut left_chains: Vec<Vec<PathEl>> = Vec::with_capacity(segments.len());
        let mut right_chains: Vec<Vec<PathEl>> = Vec::with_capacity(segments.len());

        let is_closed = path
            .elements()
            .iter()
            .any(|el| matches!(el, PathEl::ClosePath));

        // --- 1. Generate Offsets ---
        for (i, seg) in segments.iter().enumerate() {
            // Determine widths for this segment
            let w0 = widths[i % widths.len()];
            let w1 = widths[(i + 1) % widths.len()];

            // Convert segment to cubic (normalize inputs)
            let cubic = match seg {
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
                PathSeg::Cubic(c) => *c,
            };

            // Calculate Left Side (Positive width)
            let mut left_res = BezPath::new();
            // Note: offset_cubic_variable must be imported/available
            offset_cubic_variable(cubic, w0, w1, self.tolerance, &mut left_res);
            // Collect elements, stripping the initial MoveTo for all but the very first logic (handled later)
            left_chains.push(left_res.elements().to_vec());

            // Calculate Right Side (Negative width -> flips side)
            let mut right_res = BezPath::new();
            offset_cubic_variable(cubic, -w0, -w1, self.tolerance, &mut right_res);
            right_chains.push(right_res.elements().to_vec());
        }

        // --- 2. Stitching ---
        let mut result = BezPath::new();

        // -- A. Left Side (Forward) --
        for (i, chain) in left_chains.iter().enumerate() {
            for (j, el) in chain.iter().enumerate() {
                match el {
                    PathEl::MoveTo(p) => {
                        // Only the very first point of the entire path needs a MoveTo.
                        // For subsequent segments, the "Join" logic handles the connection.
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

            // JOIN LOGIC:
            // If this is not the last segment, join to the start of the next segment.
            if i < left_chains.len() - 1 || is_closed {
                let next_idx = (i + 1) % left_chains.len();
                let next_start = get_start_point(&left_chains[next_idx]);
                // This creates a "Bevel" join (straight line) between the end of this offset
                // and the start of the next. This handles sharp corners robustly.
                result.line_to(next_start);
            }
        }

        // -- B. Tip Cap --
        // Connect End of Left to End of Right.
        // The Right path was generated "Forward", so its last point is the tip.
        if !is_closed {
            if let Some(last_chain) = right_chains.last() {
                if let Some(end_point) = get_end_point(last_chain) {
                    result.line_to(end_point);
                }
            }
        } else {
            // For closed paths, we just line to the "start" of the right path's last segment
            // but actually, we handle the closure via the loop structure.
            // We need to jump to the right side mesh.
            if let Some(last_chain) = right_chains.last() {
                if let Some(end_point) = get_end_point(last_chain) {
                    result.line_to(end_point);
                }
            }
        }

        // -- C. Right Side (Reverse) --
        // We iterate the chains in reverse order (Last segment -> First segment)
        for i in (0..right_chains.len()).rev() {
            let chain = &right_chains[i];

            // We need to reverse the geometry of this chain.
            // Convert chain to segments, reverse them, and append.
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

            // JOIN LOGIC (Right side):
            // Connect to the END of the previous segment (which is the next in our reverse loop).
            if i > 0 || is_closed {
                let next_idx = if i == 0 {
                    right_chains.len() - 1
                } else {
                    i - 1
                };
                // Since we are going backwards, we join to the END of the next chain in the loop
                let next_end = get_end_point(&right_chains[next_idx]).unwrap();
                result.line_to(next_end);
            }
        }

        // -- D. Start Cap / Close --
        result.close_path();

        result
    }
}

// --- Helpers ---

fn get_start_point(chain: &[PathEl]) -> Point {
    match chain.first() {
        Some(PathEl::MoveTo(p)) => *p,
        // Should logically always be a MoveTo first from offset_cubic
        _ => Point::ZERO,
    }
}

fn get_end_point(chain: &[PathEl]) -> Option<Point> {
    match chain.last() {
        Some(PathEl::MoveTo(p)) | Some(PathEl::LineTo(p)) => Some(*p),
        Some(PathEl::QuadTo(_, p)) => Some(*p),
        Some(PathEl::CurveTo(_, _, p)) => Some(*p),
        _ => None,
    }
}

/// Helper to turn a slice of PathEl into an iterator of PathSeg
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
                    continue; // Skip MoveTo, look for next segment
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

        let widths = vec![5.0, 15.0]; // Width varies from 5 to 15
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

        let widths = vec![5.0, 50.0]; // Large width variation
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
