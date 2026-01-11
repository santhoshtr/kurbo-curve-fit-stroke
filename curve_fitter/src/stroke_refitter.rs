//! Stroke refitting module
//!
//! This module provides functionality to refit stroked path outlines by extracting
//! on-curve points with their incoming/outgoing tangent angles and running the
//! curve fitting algorithm to produce cleaner, optimized curves.

use crate::{InputPoint, PointType, fit_curve};
use kurbo::{BezPath, PathEl, Point, Vec2};

const CORNER_THRESHOLD_DEGREES: f64 = 10.0;
const G1_SMOOTH_THRESHOLD_DEGREES: f64 = 7.0;
const DEDUP_EPSILON: f64 = 1e-6;
const EPS: f64 = 1e-12;
const SKELETON_MATCH_TOLERANCE: f64 = 1.0;

// ============================================================================
// Skeleton Angle Preservation Data Structures
// ============================================================================

/// Stores pre-computed information about the skeleton path for angle preservation
///
/// This struct holds all necessary data to match outline points back to skeleton
/// locations. It should be created once before stroking, then reused for multiple
/// refit operations on the same skeleton.
///
/// # Fields
/// * `segments` - Pre-computed skeleton segments with geometry and width info
/// * `angles_at_points` - Incoming/outgoing angles at each on-curve skeleton point
/// * `is_closed` - Whether the original skeleton path was closed
#[derive(Debug, Clone)]
pub struct SkeletonInfo {
    /// Pre-computed skeleton segments for fast nearest-point queries
    /// Length = number of segments in original path
    segments: Vec<SkeletonSegment>,

    /// Incoming and outgoing angles at each on-curve skeleton point (degrees)
    /// Index corresponds to on-curve points in the original skeleton path
    /// Length = number of on-curve points
    angles_at_points: Vec<(Option<f64>, Option<f64>)>,

    /// Whether the skeleton path is closed
    is_closed: bool,
}

/// One segment of the skeleton path with pre-computed geometry
///
/// A segment is a single path element (LineTo, QuadTo, or CurveTo).
/// We cache geometric information to avoid recalculating during matching.
#[derive(Clone, Debug)]
struct SkeletonSegment {
    /// Index of this segment in the original path
    /// Corresponds to the on-curve point where this segment ends
    segment_index: usize,

    /// Start point of this segment (on-curve)
    start_point: Point,

    /// End point of this segment (on-curve)
    end_point: Point,

    /// Stroke width at segment start
    /// Used for offset distance validation
    start_width: f64,

    /// Stroke width at segment end
    /// Used for offset distance validation and interpolation
    end_width: f64,

    /// Tangent vector at segment start (cached for angle extraction)
    start_tangent: Vec2,

    /// Tangent vector at segment end (cached for angle extraction)
    end_tangent: Vec2,

    /// The actual path element (LineTo, QuadTo, CurveTo)
    /// Stored for derivative calculation at arbitrary parameters
    element: PathEl,
}

/// Result of matching an outline point back to skeleton geometry
///
/// This struct contains all information about where an outline point came from
/// in the skeleton, whether the match is confident, and what angle to apply.
///
/// An outline point "matches" a skeleton location if:
/// - The perpendicular distance from outline point to skeleton ≈ expected_offset
/// - The expected_offset is interpolated from skeleton widths
/// - The tolerance for this match is 1.0 unit
#[derive(Debug, Clone)]
pub struct OutlineSkeletonMatch {
    /// Index of the skeleton segment this outline point came from
    /// Index refers to segments in SkeletonInfo
    pub segment_index: usize,

    /// Parameter along the skeleton segment [0.0, 1.0]
    /// 0.0 = segment start (on-curve point)
    /// 1.0 = segment end (on-curve point)
    /// 0.0 < t < 1.0 = between skeleton on-curve points
    pub segment_parameter_t: f64,

    /// Interpolated skeleton angle at the matched location (degrees)
    ///
    /// Extraction logic:
    /// - If t ≈ 0.0: use skeleton point's outgoing angle
    /// - If t ≈ 1.0: use skeleton point's incoming angle
    /// - If 0.0 < t < 1.0: use segment tangent angle at parameter t
    ///
    /// None if unable to compute (e.g., degenerate segment)
    pub skeleton_angle: Option<f64>,

    /// Actual perpendicular distance from outline point to matched skeleton point
    /// Used for validation and debugging
    pub distance_to_skeleton: f64,

    /// Whether this is a "confident" match
    ///
    /// Confident if: |distance_to_skeleton - expected_offset| <= tolerance (1.0)
    ///
    /// Only confident matches have their skeleton angles applied to outline points.
    /// Non-confident matches are silently ignored (outline angles preserved).
    pub is_confident_match: bool,

    /// Expected stroke offset at the matched skeleton location
    /// Calculated by interpolating widths at parameter t
    pub expected_offset: f64,
}

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

/// Average two angles while respecting circular angle space
/// Returns None if either angle is None
///
/// Uses the shorter arc between the two angles for averaging.
///
/// # Examples
/// - `average_angles_degrees(Some(0.0), Some(90.0))` returns `Some(45.0)`
/// - `average_angles_degrees(Some(350.0), Some(10.0))` returns `Some(0.0)`
/// - `average_angles_degrees(None, Some(45.0))` returns `None`
fn average_angles_degrees(angle1: Option<f64>, angle2: Option<f64>) -> Option<f64> {
    match (angle1, angle2) {
        (Some(a1), Some(a2)) => {
            // Calculate the difference using the shorter arc
            let diff = angle_difference_degrees(a1, a2);
            // Average by starting from a1 and going halfway towards a2
            let avg = normalize_angle_degrees(a1 + diff / 2.0);
            Some(avg)
        }
        _ => None,
    }
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
// G1 Continuity Smoothing
// ============================================================================

/// Apply G1 continuity smoothing to input points
///
/// For points marked as Smooth, if the angle difference between incoming
/// and outgoing tangents is within the threshold, averages them to enforce
/// perfect G1 continuity.
///
/// # Arguments
/// * `input_points` - Points to smooth (modified in-place)
/// * `threshold_degrees` - Max angle difference to trigger smoothing (4.0 recommended)
///
/// # Returns
/// * Number of points that were smoothed (for validation)
fn g1_smooth(input_points: &mut Vec<InputPoint>, threshold_degrees: f64) -> usize {
    let mut smoothed_count = 0;

    for point in input_points.iter_mut() {
        // Only smooth points explicitly marked as Smooth
        if !matches!(point.point_type, PointType::Smooth) {
            continue;
        }

        // Both angles must be defined to smooth
        if let (Some(incoming), Some(outgoing)) = (point.incoming_angle, point.outgoing_angle) {
            let angle_diff = angle_difference_degrees(incoming, outgoing).abs();
            dbg!(&point);
            dbg!(&angle_diff);
            // If difference is within threshold, average them
            if angle_diff <= threshold_degrees {
                if let Some(averaged) = average_angles_degrees(Some(incoming), Some(outgoing)) {
                    point.incoming_angle = Some(averaged);
                    point.outgoing_angle = Some(averaged);
                    smoothed_count += 1;
                }
            }
        }
    }

    smoothed_count
}

/// Validate that G1 smoothing was applied correctly
///
/// Checks that all Smooth points with both angles defined have matching
/// incoming and outgoing angles (within floating-point precision tolerance)
fn validate_g1_smooth(input_points: &[InputPoint]) -> Result<(), String> {
    const ANGLE_TOLERANCE_DEGREES: f64 = 1e-9;

    for (i, point) in input_points.iter().enumerate() {
        if !matches!(point.point_type, PointType::Smooth) {
            continue;
        }

        match (point.incoming_angle, point.outgoing_angle) {
            (Some(incoming), Some(outgoing)) => {
                let diff = angle_difference_degrees(incoming, outgoing).abs();
                if diff > ANGLE_TOLERANCE_DEGREES {
                    return Err(format!(
                        "Point {} marked Smooth but has angle mismatch: {} ≠ {} (diff: {:.9}°)",
                        i, incoming, outgoing, diff
                    ));
                }
            }
            (None, None) => {
                // Both None is acceptable for endpoints
            }
            _ => {
                // One None, one Some would indicate a logic error
                // but don't fail—just skip validation
            }
        }
    }

    Ok(())
}

// ============================================================================
// Skeleton Angle Preservation - Helper Functions
// ============================================================================

/// Convert a tangent vector to an angle in degrees
///
/// The tangent vector points in the direction of motion along a curve.
/// We convert this to an angle measured from East (positive X-axis),
/// counterclockwise, in degrees.
///
/// # Formula
///
/// angle = atan2(tangent.y, tangent.x) * 180 / π
///
/// # Returns
///
/// * `Some(angle)` - Successfully computed angle in degrees [-180, 180]
/// * `None` - Degenerate tangent (zero-length vector)
fn tangent_to_angle_degrees_skeleton(tangent: Vec2) -> Option<f64> {
    if tangent.hypot2() < EPS {
        return None;
    }
    Some(tangent.y.atan2(tangent.x).to_degrees())
}

/// Interpolate stroke width at a parameter along a skeleton segment
///
/// Widths are specified only at skeleton on-curve points. Between points,
/// the width is linearly interpolated. This function computes the interpolated
/// width at any parameter along a segment.
///
/// # Formula
///
/// w(t) = w_start * (1 - t) + w_end * t
///
/// where:
/// - t ∈ [0, 1] is the parameter along the segment
/// - w_start is the width at segment start
/// - w_end is the width at segment end
///
/// # Arguments
///
/// * `segment`: The skeleton segment
/// * `t`: Parameter [0, 1] along the segment
///
/// # Returns
///
/// Interpolated width (always positive)
fn interpolate_width_at_parameter(segment: &SkeletonSegment, t: f64) -> f64 {
    let t_clamped = t.clamp(0.0, 1.0);
    segment.start_width * (1.0 - t_clamped) + segment.end_width * t_clamped
}

/// Compute the tangent vector of a skeleton segment at a specific parameter
///
/// The tangent vector represents the direction of motion along the segment.
/// For curve fitting, we convert this to an angle via atan2.
///
/// # Algorithm
///
/// **For LineTo(p1):**
///   - Tangent is constant: direction from p0 to p1
///   - Derivative: p1 - p0
///
/// **For QuadTo(cp, p2):**
///   - Quadratic Bézier derivative: B'(t) = 2[(1-t)(cp-p0) + t(p2-cp)]
///   - Compute at given parameter t
///
/// **For CurveTo(cp1, cp2, p3):**
///   - Cubic Bézier derivative:
///     B'(t) = 3[(1-t)²(cp1-p0) + 2(1-t)t(cp2-cp1) + t²(p3-cp2)]
///   - Compute at given parameter t
///
/// # Returns
///
/// Tangent vector (may be zero-length for degenerate segments)
fn compute_segment_tangent_at_parameter(segment: &SkeletonSegment, t: f64) -> Vec2 {
    let p0 = segment.start_point;

    match segment.element {
        PathEl::LineTo(p1) => {
            // For line, tangent is constant
            p1 - p0
        }
        PathEl::QuadTo(cp, p2) => {
            // Quadratic Bézier derivative
            // B'(t) = 2(1-t)(cp - p0) + 2*t(p2 - cp)
            let deriv = 2.0 * ((1.0 - t) * (cp - p0) + t * (p2 - cp));
            deriv
        }
        PathEl::CurveTo(cp1, cp2, p3) => {
            // Cubic Bézier derivative
            // B'(t) = 3(1-t)²(cp1-p0) + 6(1-t)t(cp2-cp1) + 3t²(p3-cp2)
            let t2 = t * t;
            let one_minus_t = 1.0 - t;
            let one_minus_t2 = one_minus_t * one_minus_t;

            let deriv = 3.0
                * (one_minus_t2 * (cp1 - p0)
                    + 2.0 * one_minus_t * t * (cp2 - cp1)
                    + t2 * (p3 - cp2));
            deriv
        }
        _ => Vec2::ZERO,
    }
}

/// Find the closest point on a skeleton segment to an outline point
///
/// For the outline point to be matched to a skeleton location, we need to find
/// the point on the skeleton that is closest to it. This is the perpendicular
/// projection for lines, and requires numerical search for curves.
///
/// # Returns
///
/// * `(closest_point, parameter_t, distance)`
///
/// where:
/// - `closest_point`: The point on the skeleton segment closest to query_point
/// - `parameter_t`: Parameter [0, 1] along the segment where closest_point is
/// - `distance`: Perpendicular distance from query_point to skeleton_point
///
/// # Algorithm
///
/// For **LineTo**:
///   - Perpendicular projection is trivial
///   - Calculate t and clamp to [0, 1]
///
/// For **QuadTo/CurveTo**:
///   - Use numerical search (sample multiple t values)
///   - Find t that minimizes distance
fn closest_point_on_segment(segment: &SkeletonSegment, query_point: Point) -> (Point, f64, f64) {
    let p0 = segment.start_point;
    let p1 = segment.end_point;

    match segment.element {
        PathEl::LineTo(_) => {
            // Simple line-point distance
            let line_vec = p1 - p0;
            let line_len_sq = line_vec.hypot2();

            if line_len_sq < EPS {
                // Degenerate line segment
                return (p0, 0.0, (query_point - p0).hypot());
            }

            // Project query_point onto the line
            let to_query = query_point - p0;
            let t = (to_query.dot(line_vec) / line_len_sq).clamp(0.0, 1.0);
            let closest = p0 + line_vec * t;
            let distance = (query_point - closest).hypot();

            (closest, t, distance)
        }
        PathEl::QuadTo(cp, p2) | PathEl::CurveTo(cp, _, p2) => {
            // Numerical search: sample the curve at multiple points
            // and find the parameter with minimum distance
            const SAMPLES: usize = 20;
            let mut best_t = 0.0;
            let mut best_dist = f64::INFINITY;
            let mut best_point = p0;

            for i in 0..=SAMPLES {
                let t = (i as f64) / (SAMPLES as f64);

                // Evaluate curve at parameter t
                let curve_point = match segment.element {
                    PathEl::QuadTo(cp, p2) => {
                        // Quadratic Bézier: B(t) = (1-t)²p0 + 2(1-t)t*cp + t²p2
                        let one_minus_t = 1.0 - t;
                        let coeff0 = one_minus_t * one_minus_t;
                        let coeff1 = 2.0 * one_minus_t * t;
                        let coeff2 = t * t;
                        let p0_vec = p0.to_vec2();
                        let cp_vec = cp.to_vec2();
                        let p2_vec = p2.to_vec2();
                        let result_vec = coeff0 * p0_vec + coeff1 * cp_vec + coeff2 * p2_vec;
                        Point::new(result_vec.x, result_vec.y)
                    }
                    PathEl::CurveTo(cp1, cp2, p3) => {
                        // Cubic Bézier: B(t) = (1-t)³p0 + 3(1-t)²t*cp1 + 3(1-t)t²*cp2 + t³p3
                        let one_minus_t = 1.0 - t;
                        let one_minus_t2 = one_minus_t * one_minus_t;
                        let one_minus_t3 = one_minus_t2 * one_minus_t;
                        let t2 = t * t;
                        let t3 = t2 * t;

                        let coeff0 = one_minus_t3;
                        let coeff1 = 3.0 * one_minus_t2 * t;
                        let coeff2 = 3.0 * one_minus_t * t2;
                        let coeff3 = t3;

                        let p0_vec = p0.to_vec2();
                        let cp1_vec = cp1.to_vec2();
                        let cp2_vec = cp2.to_vec2();
                        let p3_vec = p3.to_vec2();
                        let result_vec =
                            coeff0 * p0_vec + coeff1 * cp1_vec + coeff2 * cp2_vec + coeff3 * p3_vec;
                        Point::new(result_vec.x, result_vec.y)
                    }
                    _ => p0,
                };

                let dist = (query_point - curve_point).hypot();
                if dist < best_dist {
                    best_dist = dist;
                    best_t = t;
                    best_point = curve_point;
                }
            }

            (best_point, best_t, best_dist)
        }
        _ => {
            // Fallback for other elements
            (p0, 0.0, (query_point - p0).hypot())
        }
    }
}

/// Extract the skeleton angle at a specific location on a skeleton segment
///
/// Given a skeleton segment and a parameter along that segment, this function
/// computes the appropriate angle to use for curve fitting.
///
/// # Algorithm
///
/// Three cases:
///
/// 1. **At skeleton start point** (t ≈ 0.0):
///    - Use the outgoing angle of the skeleton point
///    - This is the direction the skeleton leaves this point
///
/// 2. **At skeleton end point** (t ≈ 1.0):
///    - Use the incoming angle of the skeleton point
///    - This is the direction the skeleton arrives at this point
///
/// 3. **Between skeleton points** (0.0 < t < 1.0):
///    - Calculate the segment tangent at parameter t
///    - Convert tangent vector to angle: atan2(tan.y, tan.x)
///    - This represents the local curve direction at that location
///
/// # Arguments
///
/// * `segment_index` - Index into SkeletonInfo.segments
/// * `segment_parameter_t` - Parameter along segment [0.0, 1.0]
/// * `skeleton_info` - Pre-registered skeleton
///
/// # Returns
///
/// * `Some(angle_degrees)` - Successfully computed angle
/// * `None` - Unable to compute (degenerate tangent, invalid index, etc.)
///
/// # Notes
///
/// - Angles are in degrees, normalized to [-180, 180]
/// - For between-point locations, uses segment derivative (tangent vector)
/// - Degenerate segments (zero-length tangent) return None
fn extract_skeleton_angle_at_parameter(
    segment_index: usize,
    segment_parameter_t: f64,
    skeleton_info: &SkeletonInfo,
) -> Option<f64> {
    if segment_index >= skeleton_info.segments.len() {
        return None;
    }

    let segment = &skeleton_info.segments[segment_index];
    const T_EPSILON: f64 = 0.05; // Threshold to consider t "near" 0 or 1

    // Case 1: Near segment start (t ≈ 0.0) - use skeleton point's outgoing angle
    if segment_parameter_t < T_EPSILON {
        // This is the start point of the segment
        // Use the outgoing angle from the angles_at_points array
        if segment_index < skeleton_info.angles_at_points.len() {
            return skeleton_info.angles_at_points[segment_index].1; // outgoing
        }
    }

    // Case 2: Near segment end (t ≈ 1.0) - use skeleton point's incoming angle
    if segment_parameter_t > (1.0 - T_EPSILON) {
        // This is the end point of the segment
        // Use the incoming angle from the next skeleton point
        let next_point_index = segment_index + 1;
        if next_point_index < skeleton_info.angles_at_points.len() {
            return skeleton_info.angles_at_points[next_point_index].0; // incoming
        }
    }

    // Case 3: Between skeleton points - compute tangent at parameter t
    let tangent = compute_segment_tangent_at_parameter(segment, segment_parameter_t);
    tangent_to_angle_degrees_skeleton(tangent)
}

// ============================================================================
// Skeleton Angle Preservation - Main Functions
// ============================================================================

/// Register a skeleton path for angle preservation during stroke refitting
///
/// Call this BEFORE stroking to prepare the skeleton for later use in
/// `refit_stroke_with_skeleton()`. This function pre-computes all necessary
/// geometric information to make matching fast and efficient.
///
/// # Arguments
///
/// * `skeleton_path` - The original curve skeleton as a BezPath
///   - Should contain only move, line, quad, and curve elements
///   - No ClosePath in the middle; only at the very end if closed
///
/// * `skeleton_angles` - InputPoints defining the skeleton's on-curve points
///   - Must have exactly one InputPoint per on-curve point in skeleton_path
///   - On-curve points are: start of path + endpoint of each line/quad/curve
///   - Each InputPoint includes incoming_angle and outgoing_angle
///
/// * `widths` - Stroke width at each skeleton on-curve point
///   - Length must equal number of on-curve points
///   - For constant width: all values the same
///   - For variable width: different values interpolated linearly between points
///
/// * `is_closed` - Whether the skeleton path forms a closed loop
///
/// # Returns
///
/// * `Ok(SkeletonInfo)` - Successfully registered skeleton, ready for matching
/// * `Err(String)` - Invalid input (mismatched lengths, empty path, etc.)
///
/// # Examples
///
/// ```ignore
/// let skeleton = fit_curve(input_points, false)?;  // Fitted curve
/// let skeleton_info = register_skeleton_for_preservation(
///     &skeleton,
///     &input_points,  // Original input points with angles
///     &[5.0, 5.0, 5.0],  // Constant width
///     false,  // Open path
/// )?;
///
/// // Now stroke and refit using this skeleton_info...
/// let outline = stroker.stroke(&skeleton, &widths, &style);
/// let refitted = refit_stroke_with_skeleton(&outline, &skeleton_info)?;
/// ```
pub fn register_skeleton_for_preservation(
    skeleton_path: &BezPath,
    skeleton_angles: &[InputPoint],
    widths: &[f64],
    is_closed: bool,
) -> Result<SkeletonInfo, String> {
    // Count on-curve points in the skeleton path
    // On-curve points are: initial MoveTo + endpoint of each LineTo/QuadTo/CurveTo
    let elements: Vec<PathEl> = skeleton_path.iter().collect();

    if elements.is_empty() {
        return Err("Empty skeleton path".to_string());
    }

    // Count on-curve points
    let mut on_curve_count = 0;
    let mut has_move_to = false;

    for el in &elements {
        match el {
            PathEl::MoveTo(_) => {
                if !has_move_to {
                    on_curve_count += 1;
                    has_move_to = true;
                }
            }
            PathEl::LineTo(_) | PathEl::QuadTo(_, _) | PathEl::CurveTo(_, _, _) => {
                on_curve_count += 1;
            }
            PathEl::ClosePath => {
                // ClosePath doesn't add an on-curve point
            }
        }
    }

    // Validate input lengths
    if skeleton_angles.len() != on_curve_count {
        return Err(format!(
            "Skeleton angles length mismatch: expected {} on-curve points, got {} angles",
            on_curve_count,
            skeleton_angles.len()
        ));
    }

    if widths.len() != on_curve_count {
        return Err(format!(
            "Widths length mismatch: expected {} on-curve points, got {} widths",
            on_curve_count,
            widths.len()
        ));
    }

    // Validate widths are positive
    for (i, &w) in widths.iter().enumerate() {
        if w <= 0.0 {
            return Err(format!("Width at index {} must be positive, got {}", i, w));
        }
    }

    // Extract angles from InputPoints
    let angles_at_points: Vec<(Option<f64>, Option<f64>)> = skeleton_angles
        .iter()
        .map(|inp| (inp.incoming_angle, inp.outgoing_angle))
        .collect();

    // Build skeleton segments
    let mut segments = Vec::new();
    let mut current_pos = Point::ZERO;
    let mut point_index = 0;

    for el in elements {
        match el {
            PathEl::MoveTo(p) => {
                current_pos = p;
                point_index += 1;
            }
            PathEl::LineTo(p1) => {
                if point_index >= widths.len() {
                    break;
                }

                let start_width = if point_index > 0 {
                    widths[point_index - 1]
                } else {
                    widths[0]
                };
                let end_width = widths[point_index];

                let start_tan = p1 - current_pos;
                let end_tan = p1 - current_pos;

                segments.push(SkeletonSegment {
                    segment_index: point_index - 1,
                    start_point: current_pos,
                    end_point: p1,
                    start_width,
                    end_width,
                    start_tangent: start_tan,
                    end_tangent: end_tan,
                    element: el.clone(),
                });

                current_pos = p1;
                point_index += 1;
            }
            PathEl::QuadTo(cp, p2) => {
                if point_index >= widths.len() {
                    break;
                }

                let start_width = if point_index > 0 {
                    widths[point_index - 1]
                } else {
                    widths[0]
                };
                let end_width = widths[point_index];

                let start_tan = cp - current_pos;
                let end_tan = p2 - cp;

                segments.push(SkeletonSegment {
                    segment_index: point_index - 1,
                    start_point: current_pos,
                    end_point: p2,
                    start_width,
                    end_width,
                    start_tangent: start_tan,
                    end_tangent: end_tan,
                    element: el.clone(),
                });

                current_pos = p2;
                point_index += 1;
            }
            PathEl::CurveTo(cp1, cp2, p3) => {
                if point_index >= widths.len() {
                    break;
                }

                let start_width = if point_index > 0 {
                    widths[point_index - 1]
                } else {
                    widths[0]
                };
                let end_width = widths[point_index];

                let start_tan = cp1 - current_pos;
                let end_tan = p3 - cp2;

                segments.push(SkeletonSegment {
                    segment_index: point_index - 1,
                    start_point: current_pos,
                    end_point: p3,
                    start_width,
                    end_width,
                    start_tangent: start_tan,
                    end_tangent: end_tan,
                    element: el.clone(),
                });

                current_pos = p3;
                point_index += 1;
            }
            PathEl::ClosePath => {
                // ClosePath handled, no segment created
            }
        }
    }

    Ok(SkeletonInfo {
        segments,
        angles_at_points,
        is_closed,
    })
}

/// Match each outline point to its source skeleton location using offset distance
///
/// For each point extracted from the stroke outline, this function finds the
/// nearest skeleton geometry and validates that the distance matches the expected
/// stroke offset. This establishes an unambiguous mapping from outline geometry
/// back to the original skeleton.
///
/// # Algorithm
///
/// For each outline_point:
/// 1. Linear search: find nearest skeleton segment (perpendicular distance)
/// 2. Calculate the parameter t along that segment
/// 3. Interpolate the stroke width w(t) at parameter t
/// 4. Calculate expected_offset = w(t) / 2.0
/// 5. Measure actual_distance from outline_point to nearest skeleton point
/// 6. IF |actual_distance - expected_offset| <= tolerance (1.0):
///    - Match is CONFIDENT, extract skeleton angle
///    ELSE:
///    - Match is NOT CONFIDENT, return None for this point
///
/// # Arguments
///
/// * `outline_points` - Points extracted from the stroked outline
///   - These are InputPoints with positions and angles extracted from outline
///   - May include points from caps, joins, and degenerate segments
///
/// * `skeleton_info` - Pre-registered skeleton from `register_skeleton_for_preservation()`
///
/// * `distance_tolerance` - How close outline point must be to expected offset (units)
///   - Recommended: 1.0
///   - Accounts for numerical precision and rasterization artifacts
///
/// # Returns
///
/// Vector of matches, same length as outline_points
/// - `Some(OutlineSkeletonMatch)` - Confident match found
/// - `None` - No confident match (point from cap/join/degenerate segment)
///
/// # Notes
///
/// - Non-confident matches are simply None; no error is raised
/// - This allows robust handling of caps, joins, and artifacts
/// - Points with None matches keep their outline-extracted angles
fn match_outline_points_to_skeleton(
    outline_points: &[InputPoint],
    skeleton_info: &SkeletonInfo,
    distance_tolerance: f64,
) -> Vec<Option<OutlineSkeletonMatch>> {
    let mut matches = Vec::new();

    for outline_point in outline_points {
        let query_point = Point::new(outline_point.x, outline_point.y);

        // Linear search: find nearest skeleton segment
        let mut best_segment_index = 0;
        let mut best_t = 0.0;
        let mut best_distance = f64::INFINITY;

        for (i, segment) in skeleton_info.segments.iter().enumerate() {
            let (_closest_point, t, distance) = closest_point_on_segment(segment, query_point);

            if distance < best_distance {
                best_distance = distance;
                best_t = t;
                best_segment_index = i;
            }
        }

        // No segments available
        if skeleton_info.segments.is_empty() {
            matches.push(None);
            continue;
        }

        let best_segment = &skeleton_info.segments[best_segment_index];

        // Interpolate width at parameter t
        let interpolated_width = interpolate_width_at_parameter(best_segment, best_t);
        let expected_offset = interpolated_width / 2.0;

        // Validate distance matches expected offset
        let distance_delta = (best_distance - expected_offset).abs();
        let is_confident = distance_delta <= distance_tolerance;

        // Extract skeleton angle if confident match
        let skeleton_angle = if is_confident {
            extract_skeleton_angle_at_parameter(best_segment_index, best_t, skeleton_info)
        } else {
            None
        };

        // Create match result
        let match_result = if is_confident && skeleton_angle.is_some() {
            Some(OutlineSkeletonMatch {
                segment_index: best_segment_index,
                segment_parameter_t: best_t,
                skeleton_angle,
                distance_to_skeleton: best_distance,
                is_confident_match: true,
                expected_offset,
            })
        } else {
            None
        };

        matches.push(match_result);
    }

    matches
}

/// Apply skeleton angles to outline points based on confident matches
///
/// This function updates the outline points with angles from the skeleton,
/// but ONLY for confident matches. Points without confident matches keep
/// their original outline-extracted angles.
///
/// For each outline point with a confident skeleton match:
/// - Set incoming_angle = skeleton_angle
/// - Set outgoing_angle = skeleton_angle
/// - Preserve point_type (Smooth or Corner as-is)
///
/// Points with uncertain matches (distance mismatch) are silently ignored,
/// preserving their outline-extracted angles. This provides graceful
/// degradation for artifacts like caps, joins, and degenerate segments.
///
/// # Arguments
///
/// * `outline_points` - Points to modify (in-place)
///   - Will be updated with skeleton angles where matches are confident
///
/// * `skeleton_matches` - Matches from `match_outline_points_to_skeleton()`
///   - Same length as outline_points
///   - None values are skipped silently
///
/// # Returns
///
/// Count of points modified (for debugging/monitoring)
fn preserve_skeleton_angles_in_outline(
    outline_points: &mut [InputPoint],
    skeleton_matches: &[Option<OutlineSkeletonMatch>],
) -> usize {
    let mut modified_count = 0;

    for (outline_pt, skeleton_match) in outline_points.iter_mut().zip(skeleton_matches) {
        if let Some(match_info) = skeleton_match {
            // Only apply skeleton angle if this is a confident match
            if match_info.is_confident_match {
                if let Some(angle) = match_info.skeleton_angle {
                    outline_pt.incoming_angle = Some(angle);
                    outline_pt.outgoing_angle = Some(angle);
                    modified_count += 1;
                }
            }
        }
    }

    modified_count
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
        let mut input_points =
            subpath_to_input_points(&subpath, CORNER_THRESHOLD_DEGREES, DEDUP_EPSILON);

        if input_points.len() < 2 {
            continue;
        }

        // Apply G1 smoothing to enforce continuity
        g1_smooth(&mut input_points, G1_SMOOTH_THRESHOLD_DEGREES);

        // Validate smoothing was applied correctly
        validate_g1_smooth(&input_points)?;

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

/// Refit a stroked outline while preserving angles from the original skeleton
///
/// This is the main entry point for stroke refitting with skeleton angle preservation.
/// It combines all the previous steps into one convenient function.
///
/// The pipeline:
/// 1. Extract subpaths from the outline
/// 2. For each subpath, extract InputPoints with outline-derived angles
/// 3. Match outline points back to skeleton using offset distance
/// 4. Preserve skeleton angles for confident matches
/// 5. Apply G1 smoothing to enforce continuity
/// 6. Validate G1 smoothing succeeded
/// 7. Fit final curve using Hobby's algorithm
///
/// If any step fails (except matching, which degrades gracefully), an error is returned.
///
/// # Arguments
///
/// * `stroke_path` - The stroked outline (BezPath from VariableStroker)
///
/// * `skeleton_info` - Pre-registered skeleton from `register_skeleton_for_preservation()`
///   - Contains geometry and angles of the original skeleton
///
/// # Returns
///
/// * `Ok(BezPath)` - Successfully refitted curve with skeleton angles preserved
/// * `Err(String)` - Error in subpath extraction, angle smoothing, or curve fitting
///
/// # Notes
///
/// - Matching failures (points without skeleton correspondence) are silently
///   handled by preserving outline-extracted angles
/// - This provides graceful degradation for caps, joins, and artifacts
/// - Only confident matches (distance within 1.0 unit) get skeleton angles
/// - Non-confident points keep outline angles as fallback
///
/// # Examples
///
/// ```ignore
/// // Before stroking, register the skeleton
/// let skeleton_info = register_skeleton_for_preservation(
///     &skeleton_bezpath,
///     &skeleton_input_points,
///     &widths,
///     false,  // open path
/// )?;
///
/// // Stroke the path
/// let outline = stroker.stroke(&skeleton_bezpath, &widths, &style);
///
/// // Refit with skeleton preservation
/// let refitted = refit_stroke_with_skeleton(&outline, &skeleton_info)?;
/// ```
pub fn refit_stroke_with_skeleton(
    stroke_path: &BezPath,
    skeleton_info: &SkeletonInfo,
) -> Result<BezPath, String> {
    let subpaths = extract_subpaths(stroke_path);

    if subpaths.is_empty() {
        return Ok(BezPath::new());
    }

    let mut combined_path = BezPath::new();
    let mut is_first_subpath = true;

    for subpath in subpaths {
        let mut input_points =
            subpath_to_input_points(&subpath, CORNER_THRESHOLD_DEGREES, DEDUP_EPSILON);

        if input_points.len() < 2 {
            continue;
        }

        // NEW: Match outline points back to skeleton
        let skeleton_matches = match_outline_points_to_skeleton(
            &input_points,
            skeleton_info,
            SKELETON_MATCH_TOLERANCE,
        );

        // NEW: Preserve skeleton angles for confident matches
        let preserved_count =
            preserve_skeleton_angles_in_outline(&mut input_points, &skeleton_matches);

        // Apply G1 smoothing to enforce continuity
        g1_smooth(&mut input_points, G1_SMOOTH_THRESHOLD_DEGREES);

        // Validate smoothing was applied correctly
        validate_g1_smooth(&input_points)?;

        dbg!(&input_points);
        dbg!(preserved_count);

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

    #[test]
    fn test_average_angles_degrees_basic() {
        // Simple average
        let result = average_angles_degrees(Some(0.0), Some(90.0));
        assert!(result.is_some());
        assert!((result.unwrap() - 45.0).abs() < 1e-10);

        // Another simple case
        let result = average_angles_degrees(Some(30.0), Some(60.0));
        assert!(result.is_some());
        assert!((result.unwrap() - 45.0).abs() < 1e-10);
    }

    #[test]
    fn test_average_angles_degrees_wrapping() {
        // Wrapping case: 350° and 10° should average to 0° (going the short way around)
        let result = average_angles_degrees(Some(350.0), Some(10.0));
        assert!(result.is_some());
        let avg = result.unwrap();
        // Should be 0° (or very close, accounting for floating point)
        assert!((avg - 0.0).abs() < 0.1, "Expected ~0°, got {:.2}°", avg);

        // Reverse order should give same result
        let result = average_angles_degrees(Some(10.0), Some(350.0));
        assert!(result.is_some());
        let avg = result.unwrap();
        assert!((avg - 0.0).abs() < 0.1, "Expected ~0°, got {:.2}°", avg);

        // Another wrapping case: -175° and 175°
        // These are 10° apart, so average should be ±180°
        let result = average_angles_degrees(Some(-175.0), Some(175.0));
        assert!(result.is_some());
        let avg = result.unwrap();
        assert!((avg.abs() - 180.0).abs() < 0.1);
    }

    #[test]
    fn test_average_angles_degrees_with_none() {
        // None cases
        assert!(average_angles_degrees(None, Some(45.0)).is_none());
        assert!(average_angles_degrees(Some(45.0), None).is_none());
        assert!(average_angles_degrees(None, None).is_none());
    }

    #[test]
    fn test_g1_smooth_within_threshold() {
        let mut points = vec![InputPoint {
            x: 0.0,
            y: 0.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(45.0),
            outgoing_angle: Some(47.0), // 2° difference, within 4° threshold
        }];

        let smoothed = g1_smooth(&mut points, 4.0);
        assert_eq!(smoothed, 1);

        // Both angles should now be the average (46.0)
        assert!(points[0].incoming_angle.is_some());
        assert!(points[0].outgoing_angle.is_some());
        let inc = points[0].incoming_angle.unwrap();
        let out = points[0].outgoing_angle.unwrap();
        assert!((inc - 46.0).abs() < 1e-10);
        assert!((out - 46.0).abs() < 1e-10);
    }

    #[test]
    fn test_g1_smooth_outside_threshold() {
        let mut points = vec![InputPoint {
            x: 0.0,
            y: 0.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(45.0),
            outgoing_angle: Some(55.0), // 10° difference, outside 4° threshold
        }];

        let smoothed = g1_smooth(&mut points, 4.0);
        assert_eq!(smoothed, 0);

        // Angles should remain unchanged
        assert_eq!(points[0].incoming_angle, Some(45.0));
        assert_eq!(points[0].outgoing_angle, Some(55.0));
    }

    #[test]
    fn test_g1_smooth_ignores_corners() {
        let mut points = vec![InputPoint {
            x: 0.0,
            y: 0.0,
            point_type: PointType::Corner,
            incoming_angle: Some(45.0),
            outgoing_angle: Some(47.0), // Within threshold but it's a corner
        }];

        let smoothed = g1_smooth(&mut points, 4.0);
        assert_eq!(smoothed, 0);

        // Angles should remain unchanged (corners are not smoothed)
        assert_eq!(points[0].incoming_angle, Some(45.0));
        assert_eq!(points[0].outgoing_angle, Some(47.0));
    }

    #[test]
    fn test_g1_smooth_missing_angles() {
        let mut points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(45.0),
                outgoing_angle: None,
            },
            InputPoint {
                x: 10.0,
                y: 10.0,
                point_type: PointType::Smooth,
                incoming_angle: None,
                outgoing_angle: Some(45.0),
            },
        ];

        let smoothed = g1_smooth(&mut points, 4.0);
        assert_eq!(smoothed, 0);

        // Angles should remain unchanged
        assert_eq!(points[0].incoming_angle, Some(45.0));
        assert_eq!(points[0].outgoing_angle, None);
        assert_eq!(points[1].incoming_angle, None);
        assert_eq!(points[1].outgoing_angle, Some(45.0));
    }

    #[test]
    fn test_validate_g1_smooth_passes() {
        let points = vec![
            InputPoint {
                x: 0.0,
                y: 0.0,
                point_type: PointType::Smooth,
                incoming_angle: Some(45.0),
                outgoing_angle: Some(45.0), // Perfectly matched
            },
            InputPoint {
                x: 10.0,
                y: 10.0,
                point_type: PointType::Corner,
                incoming_angle: Some(45.0),
                outgoing_angle: Some(90.0), // Mismatched but it's a corner
            },
        ];

        assert!(validate_g1_smooth(&points).is_ok());
    }

    #[test]
    fn test_validate_g1_smooth_fails() {
        let points = vec![InputPoint {
            x: 0.0,
            y: 0.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(45.0),
            outgoing_angle: Some(47.0), // Mismatched and it's smooth
        }];

        assert!(validate_g1_smooth(&points).is_err());
    }

    #[test]
    fn test_validate_g1_smooth_with_none_angles() {
        let points = vec![InputPoint {
            x: 0.0,
            y: 0.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None, // Both None is acceptable
        }];

        assert!(validate_g1_smooth(&points).is_ok());
    }
}
