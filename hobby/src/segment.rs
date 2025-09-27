use crate::point::Point;
use std::fmt::{Display, Formatter, Result as FmtResult};

/// Represents the four points defining a cubic Bézier curve.
///
/// This structure contains the start point, two control points, and end point
/// that define a cubic Bézier curve. This is the standard representation where:
/// - `start`: The curve begins at this point
/// - `control1`: First control point, pulled from the start
/// - `control2`: Second control point, pulled toward the end
/// - `end`: The curve ends at this point
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BezierSegment {
    pub start: Point,
    pub control1: Point,
    pub control2: Point,
    pub end: Point,
}

impl BezierSegment {
    /// Create a new Bézier segment from four points
    pub fn new(start: Point, control1: Point, control2: Point, end: Point) -> Self {
        BezierSegment {
            start,
            control1,
            control2,
            end,
        }
    }
}

impl Display for BezierSegment {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        write!(
            f,
            "Bezier[{} -> {} -> {} -> {}]",
            self.start, self.control1, self.control2, self.end
        )
    }
}

/// Convert from the old tuple format for backward compatibility
impl From<(Point, Point, Point, Point)> for BezierSegment {
    fn from(tuple: (Point, Point, Point, Point)) -> Self {
        BezierSegment::new(tuple.0, tuple.1, tuple.2, tuple.3)
    }
}

/// Convert to the old tuple format for backward compatibility
impl From<BezierSegment> for (Point, Point, Point, Point) {
    fn from(segment: BezierSegment) -> Self {
        (
            segment.start,
            segment.control1,
            segment.control2,
            segment.end,
        )
    }
}
