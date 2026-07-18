//! Outline extraction: subpaths, segment tangents, and InputPoint generation.

use kurbo::{BezPath, PathEl, Point, Vec2};

use super::{EPS, is_corner, vec2_to_angle_degrees};
use crate::{InputPoint, PointType};

/// Internal representation of a path segment with tangent information
#[derive(Debug, Clone)]
pub(super) struct SegmentInfo {
    pub end_point: Point,
    pub start_tangent: Option<Vec2>, // outgoing from start
    pub end_tangent: Option<Vec2>,   // incoming to end
}

/// Represents a continuous subpath extracted from a BezPath
#[derive(Debug)]
pub(super) struct Subpath {
    pub segments: Vec<SegmentInfo>,
    pub is_closed: bool,
    pub start_point: Point,
}

impl Subpath {
    fn new(start_point: Point) -> Self {
        Self {
            segments: Vec::new(),
            is_closed: false,
            start_point,
        }
    }
}

/// Extract tangent information from a PathEl
/// Returns: (end_point, start_tangent, end_tangent)
fn extract_tangent_from_segment(
    el: &PathEl,
    current_pos: Point,
) -> (Point, Option<Vec2>, Option<Vec2>) {
    match el {
        PathEl::MoveTo(p) => (*p, None, None),
        PathEl::LineTo(p) => {
            let tangent = *p - current_pos;
            let tangent = if tangent.hypot2() > EPS {
                Some(tangent)
            } else {
                None
            };
            (*p, tangent, tangent)
        }
        PathEl::QuadTo(cp, p) => {
            let start_tangent = *cp - current_pos;
            let end_tangent = *p - *cp;
            let start_tangent = if start_tangent.hypot2() > EPS {
                Some(start_tangent)
            } else {
                None
            };
            let end_tangent = if end_tangent.hypot2() > EPS {
                Some(end_tangent)
            } else {
                None
            };
            (*p, start_tangent, end_tangent)
        }
        PathEl::CurveTo(cp1, cp2, p) => {
            let start_tangent = *cp1 - current_pos;
            let end_tangent = *p - *cp2;
            let start_tangent = if start_tangent.hypot2() > EPS {
                Some(start_tangent)
            } else {
                None
            };
            let end_tangent = if end_tangent.hypot2() > EPS {
                Some(end_tangent)
            } else {
                None
            };
            (*p, start_tangent, end_tangent)
        }
        PathEl::ClosePath => {
            // ClosePath handled separately in extract_subpaths
            (current_pos, None, None)
        }
    }
}

/// Extract all subpaths from a BezPath
pub(super) fn extract_subpaths(stroke_path: &BezPath) -> Vec<Subpath> {
    let mut subpaths = Vec::new();
    let mut current_subpath: Option<Subpath> = None;
    let mut current_pos = Point::ZERO;

    for el in stroke_path.iter() {
        match el {
            PathEl::MoveTo(p) => {
                // Save current subpath if it exists
                if let Some(subpath) = current_subpath.take()
                    && !subpath.segments.is_empty()
                {
                    subpaths.push(subpath);
                }
                // Start new subpath
                current_subpath = Some(Subpath::new(p));
                current_pos = p;
            }
            PathEl::LineTo(_) | PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _) => {
                if let Some(ref mut subpath) = current_subpath {
                    let (end_point, start_tangent, end_tangent) =
                        extract_tangent_from_segment(&el, current_pos);

                    let segment = SegmentInfo {
                        end_point,
                        start_tangent,
                        end_tangent,
                    };

                    subpath.segments.push(segment);
                    current_pos = end_point;
                }
            }
            PathEl::ClosePath => {
                if let Some(ref mut subpath) = current_subpath {
                    subpath.is_closed = true;

                    // Create closing segment if needed
                    let start = subpath.start_point;
                    if (current_pos - start).hypot2() > EPS {
                        let closing_vec = start - current_pos;
                        let segment = SegmentInfo {
                            end_point: start,
                            start_tangent: Some(closing_vec),
                            end_tangent: Some(closing_vec),
                        };
                        subpath.segments.push(segment);
                    }

                    current_pos = start;
                }
            }
        }
    }

    // Save final subpath
    if let Some(mut subpath) = current_subpath
        && !subpath.segments.is_empty()
    {
        // Check if path is geometrically closed (even without explicit ClosePath)
        if !subpath.is_closed {
            let start = subpath.start_point;
            let dist_sq = (current_pos - start).hypot2();
            if dist_sq < EPS {
                // Path ends where it started - treat as closed
                subpath.is_closed = true;
            }
        }

        subpaths.push(subpath);
    }

    subpaths
}

/// Convert a subpath to InputPoints with angle information
pub(super) fn subpath_to_input_points(
    subpath: &Subpath,
    corner_threshold: f64,
    dedup_epsilon: f64,
) -> Vec<InputPoint> {
    if subpath.segments.is_empty() {
        return Vec::new();
    }

    let mut input_points: Vec<InputPoint> = Vec::new();

    // The loop below emits one point per segment END, so the subpath's start
    // point must be emitted explicitly for open subpaths. (On closed subpaths
    // it reappears as the end of the closing segment.)
    if !subpath.is_closed {
        let outgoing_tangent = subpath
            .segments
            .iter()
            .find_map(|segment| segment.start_tangent);
        input_points.push(InputPoint {
            x: subpath.start_point.x,
            y: subpath.start_point.y,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: outgoing_tangent.and_then(vec2_to_angle_degrees),
        });
    }

    for (i, segment) in subpath.segments.iter().enumerate() {
        // We're creating an InputPoint at segment.end_point.
        // For that point:
        // - incoming tangent = this segment's end_tangent (direction arriving at segment.end_point)
        // - outgoing tangent = next segment's start_tangent (direction leaving segment.end_point)

        // Get incoming tangent: direction arriving at this segment's end_point
        let incoming_tangent = segment.end_tangent;

        // Get outgoing tangent: direction leaving this point (start of next segment)
        // Skip over degenerate segments to find a valid outgoing tangent
        let outgoing_tangent = {
            let mut tangent = None;
            let mut j = i + 1;

            while j < subpath.segments.len() {
                if let Some(t) = subpath.segments[j].start_tangent {
                    tangent = Some(t);
                    break;
                }
                j += 1;
            }

            // If we didn't find a tangent and the path is closed, check from the beginning
            if tangent.is_none() && subpath.is_closed {
                for seg in &subpath.segments {
                    if let Some(t) = seg.start_tangent {
                        tangent = Some(t);
                        break;
                    }
                }
            }

            tangent
        };

        // Convert tangent vectors to angles (no flip needed - tangents already point in path direction)
        let incoming_angle = incoming_tangent.and_then(vec2_to_angle_degrees);
        let outgoing_angle = outgoing_tangent.and_then(vec2_to_angle_degrees);

        // Determine point type based on angle difference
        let point_type = if is_corner(incoming_angle, outgoing_angle, corner_threshold) {
            PointType::Corner
        } else {
            PointType::Smooth
        };

        let point = InputPoint {
            x: segment.end_point.x,
            y: segment.end_point.y,
            point_type,
            incoming_angle,
            outgoing_angle,
        };

        // Deduplication check
        if let Some(last_point) = input_points.last() {
            let dx = point.x - last_point.x;
            let dy = point.y - last_point.y;
            if dx * dx + dy * dy < dedup_epsilon * dedup_epsilon {
                continue;
            }
        }

        input_points.push(point);
    }

    // A closed subpath can yield coincident first and last points (e.g. a
    // zero-length lead-in join segment at the seam); the consecutive-pair
    // dedup above never compares the two ends. Merge them so the closed fit
    // does not see a zero-length chord: the last point carries the real
    // incoming tangent, the first the outgoing one.
    if subpath.is_closed && input_points.len() >= 2 {
        let first = &input_points[0];
        let last = &input_points[input_points.len() - 1];
        let dx = last.x - first.x;
        let dy = last.y - first.y;
        if dx * dx + dy * dy < dedup_epsilon * dedup_epsilon {
            let last = input_points.pop().unwrap();
            let first = &mut input_points[0];
            first.incoming_angle = last.incoming_angle.or(first.incoming_angle);
            first.outgoing_angle = first.outgoing_angle.or(last.outgoing_angle);
            first.point_type =
                if is_corner(first.incoming_angle, first.outgoing_angle, corner_threshold) {
                    PointType::Corner
                } else {
                    PointType::Smooth
                };
        }
    }

    // For open paths, fix endpoint angles
    if !subpath.is_closed && !input_points.is_empty() {
        // First point has no incoming angle
        input_points[0].incoming_angle = None;
        // Last point has no outgoing angle
        if let Some(last) = input_points.last_mut() {
            last.outgoing_angle = None;
        }
    }

    input_points
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_tangent_cubic() {
        let current_pos = Point::new(0.0, 0.0);
        let cp1 = Point::new(10.0, 0.0);
        let cp2 = Point::new(90.0, 100.0);
        let p3 = Point::new(100.0, 100.0);

        let el = PathEl::CurveTo(cp1, cp2, p3);
        let (end, start_tan, end_tan) = extract_tangent_from_segment(&el, current_pos);

        assert_eq!(end, p3);
        assert!(start_tan.is_some());
        assert!(end_tan.is_some());

        let start_tan = start_tan.unwrap();
        assert_eq!(start_tan, cp1 - current_pos);

        let end_tan = end_tan.unwrap();
        assert_eq!(end_tan, p3 - cp2);
    }

    #[test]
    fn test_simple_open_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((100.0, 100.0));

        let subpaths = extract_subpaths(&path);
        assert_eq!(subpaths.len(), 1);
        assert!(!subpaths[0].is_closed);
        assert_eq!(subpaths[0].segments.len(), 2);
    }

    #[test]
    fn test_closed_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((100.0, 100.0));
        path.line_to((0.0, 100.0));
        path.close_path();

        let subpaths = extract_subpaths(&path);
        assert_eq!(subpaths.len(), 1);
        assert!(subpaths[0].is_closed);
        // Should have 4 segments including the closing one
        assert_eq!(subpaths[0].segments.len(), 4);
    }

    #[test]
    fn test_multiple_subpaths() {
        let mut path = BezPath::new();
        // First subpath
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.close_path();
        // Second subpath
        path.move_to((200.0, 0.0));
        path.line_to((300.0, 0.0));
        path.close_path();

        let subpaths = extract_subpaths(&path);
        assert_eq!(subpaths.len(), 2);
        assert!(subpaths[0].is_closed);
        assert!(subpaths[1].is_closed);
    }


    #[test]
    fn test_open_subpath_keeps_first_point() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.curve_to((10.0, 10.0), (20.0, 10.0), (30.0, 0.0));
        path.curve_to((40.0, -10.0), (50.0, -10.0), (60.0, 0.0));

        let subpaths = extract_subpaths(&path);
        let points = subpath_to_input_points(&subpaths[0], 15.0, 1e-6);

        // All three on-curve points, starting with the MoveTo point
        assert_eq!(points.len(), 3);
        assert_eq!((points[0].x, points[0].y), (0.0, 0.0));
        assert_eq!(points[0].incoming_angle, None);
        assert!(points[0].outgoing_angle.is_some());
        assert_eq!((points[2].x, points[2].y), (60.0, 0.0));
        assert_eq!(points[2].outgoing_angle, None);
    }

    #[test]
    fn test_closed_subpath_merges_wraparound_duplicate() {
        // Closed ring starting with a zero-length lead-in line, as emitted by
        // the variable stroker's first join: the seam point would otherwise
        // appear both as the first extracted point (from the degenerate line,
        // with no incoming tangent) and as the last (end of the final curve).
        let mut path = BezPath::new();
        path.move_to((100.0, 0.0));
        path.line_to((100.0, 0.0)); // zero-length join sliver
        path.curve_to((100.0, 55.0), (55.0, 100.0), (0.0, 100.0));
        path.curve_to((-55.0, 100.0), (-100.0, 55.0), (-100.0, 0.0));
        path.curve_to((-100.0, -55.0), (-55.0, -100.0), (0.0, -100.0));
        path.curve_to((55.0, -100.0), (100.0, -55.0), (100.0, 0.0));
        path.close_path();

        let subpaths = extract_subpaths(&path);
        assert_eq!(subpaths.len(), 1);
        let points = subpath_to_input_points(&subpaths[0], 15.0, 1e-6);

        // 4 distinct on-curve points; the seam point appears exactly once
        assert_eq!(points.len(), 4);
        let seam = &points[0];
        assert_eq!((seam.x, seam.y), (100.0, 0.0));
        // Merged point has both tangents: incoming from the closing curve,
        // outgoing from the first real curve
        assert!(seam.incoming_angle.is_some());
        assert!(seam.outgoing_angle.is_some());
    }

    #[test]
    fn test_degenerate_tangents() {
        let current_pos = Point::new(0.0, 0.0);
        // Control point coincides with current position
        let cp1 = Point::new(0.0, 0.0);
        let cp2 = Point::new(100.0, 100.0);
        let p3 = Point::new(100.0, 100.0);

        let el = PathEl::CurveTo(cp1, cp2, p3);
        let (_, start_tan, end_tan) = extract_tangent_from_segment(&el, current_pos);

        // Start tangent should be None (zero length)
        assert!(start_tan.is_none());
        // End tangent should also be None (cp2 == p3)
        assert!(end_tan.is_none());
    }
}
