//! Stroke refitting module
//!
//! This module provides functionality to refit stroked path outlines by extracting
//! on-curve points with their incoming/outgoing tangent angles and running the
//! curve fitting algorithm to produce cleaner, optimized curves.

use crate::{InputPoint, PointType, fit_curve};
use kurbo::{BezPath, PathEl, Point, Vec2};

const CORNER_THRESHOLD_DEGREES: f64 = 10.0;
const DEDUP_EPSILON: f64 = 1e-6;
const EPS: f64 = 1e-12;

/// Internal representation of a path segment with tangent information
#[derive(Debug, Clone)]
struct SegmentInfo {
    start_point: Point,
    end_point: Point,
    start_tangent: Option<Vec2>, // outgoing from start
    end_tangent: Option<Vec2>,   // incoming to end
}

/// Represents a continuous subpath extracted from a BezPath
#[derive(Debug)]
struct Subpath {
    segments: Vec<SegmentInfo>,
    is_closed: bool,
    start_point: Point,
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

// ============================================================================
// Angle Utility Functions
// ============================================================================

/// Convert a Vec2 to an angle in degrees (0° = East, counterclockwise)
/// Returns None for zero-length vectors
fn vec2_to_angle_degrees(vec: Vec2) -> Option<f64> {
    if vec.hypot2() < EPS {
        return None;
    }
    Some(vec.y.atan2(vec.x).to_degrees())
}

/// Normalize angle to -180° to 180° range
fn normalize_angle_degrees(angle: f64) -> f64 {
    let mut normalized = angle % 360.0;
    if normalized > 180.0 {
        normalized -= 360.0;
    } else if normalized < -180.0 {
        normalized += 360.0;
    }
    normalized
}

/// Calculate the smallest angle difference between two angles
/// Returns value in -180° to 180° range
fn angle_difference_degrees(angle1: f64, angle2: f64) -> f64 {
    normalize_angle_degrees(angle2 - angle1)
}

/// Determine if the angle change between incoming and outgoing tangents
/// indicates a corner point
fn is_corner(incoming: Option<f64>, outgoing: Option<f64>, threshold: f64) -> bool {
    match (incoming, outgoing) {
        (Some(inc), Some(out)) => {
            let diff = angle_difference_degrees(inc, out).abs();
            diff > threshold
        }
        _ => false,
    }
}

// ============================================================================
// Segment Extraction Functions
// ============================================================================

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
fn extract_subpaths(stroke_path: &BezPath) -> Vec<Subpath> {
    let mut subpaths = Vec::new();
    let mut current_subpath: Option<Subpath> = None;
    let mut current_pos = Point::ZERO;

    for el in stroke_path.iter() {
        match el {
            PathEl::MoveTo(p) => {
                // Save current subpath if it exists
                if let Some(subpath) = current_subpath.take() {
                    if !subpath.segments.is_empty() {
                        subpaths.push(subpath);
                    }
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
                        start_point: current_pos,
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
                            start_point: current_pos,
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
    if let Some(mut subpath) = current_subpath {
        if !subpath.segments.is_empty() {
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
    }

    subpaths
}

// ============================================================================
// InputPoint Generation
// ============================================================================

/// Debug helper to print detailed angle extraction information
#[allow(dead_code)]
fn debug_angle_extraction(
    segment_index: usize,
    segment_end_point: Point,
    incoming_vec: Option<Vec2>,
    incoming_raw_angle: Option<f64>,
    incoming_flipped_angle: Option<f64>,
    outgoing_vec: Option<Vec2>,
    outgoing_raw_angle: Option<f64>,
    outgoing_flipped_angle: Option<f64>,
) {
    println!(
        "\n[Segment {}] Point ({:.2}, {:.2})",
        segment_index, segment_end_point.x, segment_end_point.y
    );

    if let Some(v) = incoming_vec {
        println!("  Incoming vec: ({:.4}, {:.4})", v.x, v.y);
    } else {
        println!("  Incoming vec: None");
    }

    if let Some(angle) = incoming_raw_angle {
        println!("  Incoming raw angle: {:.2}°", angle);
    } else {
        println!("  Incoming raw angle: None");
    }

    if let Some(angle) = incoming_flipped_angle {
        println!("  Incoming flipped (+180°): {:.2}°", angle);
    } else {
        println!("  Incoming flipped: None");
    }

    if let Some(v) = outgoing_vec {
        println!("  Outgoing vec: ({:.4}, {:.4})", v.x, v.y);
    } else {
        println!("  Outgoing vec: None");
    }

    if let Some(angle) = outgoing_raw_angle {
        println!("  Outgoing raw angle: {:.2}°", angle);
    } else {
        println!("  Outgoing raw angle: None");
    }

    if let Some(angle) = outgoing_flipped_angle {
        println!("  Outgoing flipped (+180°): {:.2}°", angle);
    } else {
        println!("  Outgoing flipped: None");
    }

    if let (Some(inc), Some(out)) = (incoming_flipped_angle, outgoing_flipped_angle) {
        let diff = angle_difference_degrees(inc, out).abs();
        println!("  Angle difference: {:.2}°", diff);
        if diff > 10.0 {
            println!("  → CORNER (diff > 10°)");
        } else {
            println!("  → SMOOTH (diff ≤ 10°)");
        }
    }
}

/// Convert a subpath to InputPoints with angle information
fn subpath_to_input_points(
    subpath: &Subpath,
    corner_threshold: f64,
    dedup_epsilon: f64,
) -> Vec<InputPoint> {
    if subpath.segments.is_empty() {
        return Vec::new();
    }

    let mut input_points: Vec<InputPoint> = Vec::new();

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
        let incoming_angle = incoming_tangent.and_then(|tan| vec2_to_angle_degrees(tan));
        let outgoing_angle = outgoing_tangent.and_then(|tan| vec2_to_angle_degrees(tan));

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

// ============================================================================
// Public API
// ============================================================================

/// Refits a stroked path outline by extracting on-curve points with tangent angles
/// and running the curve fitting algorithm.
///
/// This function analyzes a BezPath (typically generated from a stroke operation),
/// extracts the on-curve points along with their incoming/outgoing tangent angles,
/// and refits them using the curve fitting algorithm to produce a cleaner result.
///
/// The function preserves closed paths and properly combines multiple subpaths.
/// For example, an 'O' shaped stroke with inner and outer contours will be
/// correctly refitted while maintaining both closed subpaths.
///
/// # Arguments
/// * `stroke_path` - The stroked path outline to refit
///
/// # Returns
/// * `Ok(BezPath)` - The refitted curve combining all subpaths
/// * `Err(String)` - Error message if fitting fails (aborts on first failure)
///
/// # Constants
/// * Corner threshold: 10.0 degrees (conservative)
/// * Deduplication epsilon: 1e-6
///
/// # Examples
/// ```
/// use kurbo::BezPath;
/// use curve_fitter::refit_stroke;
///
/// // Assuming you have a stroked_path from VariableStroker
/// // let stroked_path = stroker.stroke(&path, &widths, &style);
/// // let refitted = refit_stroke(&stroked_path)?;
/// ```
pub fn refit_stroke(stroke_path: &BezPath) -> Result<BezPath, String> {
    let subpaths = extract_subpaths(stroke_path);

    if subpaths.is_empty() {
        return Ok(BezPath::new());
    }

    let mut combined_path = BezPath::new();
    let mut is_first_subpath = true;

    for subpath in subpaths {
        let input_points =
            subpath_to_input_points(&subpath, CORNER_THRESHOLD_DEGREES, DEDUP_EPSILON);

        if input_points.len() < 2 {
            continue;
        }
        dbg!(&input_points);
        // Fit the curve, abort on any failure
        let fitted_path = fit_curve(input_points, subpath.is_closed)?;

        // Combine paths while preserving closed path structure
        if is_first_subpath {
            combined_path = fitted_path;
            is_first_subpath = false;
        } else {
            // Append entire fitted_path including MoveTo and ClosePath
            for el in fitted_path.elements() {
                combined_path.push(*el);
            }
        }
    }

    Ok(combined_path)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec2_to_angle_degrees() {
        // East (positive X)
        assert_eq!(vec2_to_angle_degrees(Vec2::new(1.0, 0.0)), Some(0.0));

        // North (positive Y)
        let north = vec2_to_angle_degrees(Vec2::new(0.0, 1.0)).unwrap();
        assert!((north - 90.0).abs() < 1e-10);

        // West (negative X)
        let west = vec2_to_angle_degrees(Vec2::new(-1.0, 0.0)).unwrap();
        assert!((west.abs() - 180.0).abs() < 1e-10);

        // South (negative Y)
        let south = vec2_to_angle_degrees(Vec2::new(0.0, -1.0)).unwrap();
        assert!((south + 90.0).abs() < 1e-10);

        // Zero vector
        assert_eq!(vec2_to_angle_degrees(Vec2::ZERO), None);
    }

    #[test]
    fn test_normalize_angle_degrees() {
        assert_eq!(normalize_angle_degrees(0.0), 0.0);
        assert_eq!(normalize_angle_degrees(180.0), 180.0);
        assert_eq!(normalize_angle_degrees(-180.0), -180.0);
        assert_eq!(normalize_angle_degrees(190.0), -170.0);
        assert_eq!(normalize_angle_degrees(-190.0), 170.0);
        assert_eq!(normalize_angle_degrees(360.0), 0.0);
        assert_eq!(normalize_angle_degrees(720.0), 0.0);
    }

    #[test]
    fn test_angle_difference_degrees() {
        // Simple differences
        assert_eq!(angle_difference_degrees(0.0, 45.0), 45.0);
        assert_eq!(angle_difference_degrees(45.0, 0.0), -45.0);

        // Wrapping cases
        let diff = angle_difference_degrees(350.0, 10.0);
        assert!((diff - 20.0).abs() < 1e-10);

        let diff = angle_difference_degrees(10.0, 350.0);
        assert!((diff + 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_is_corner() {
        // Small difference - not a corner
        assert!(!is_corner(Some(0.0), Some(5.0), 10.0));

        // Large difference - is a corner
        assert!(is_corner(Some(0.0), Some(90.0), 10.0));

        // No incoming angle - not a corner
        assert!(!is_corner(None, Some(45.0), 10.0));

        // No outgoing angle - not a corner
        assert!(!is_corner(Some(45.0), None, 10.0));
    }

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
