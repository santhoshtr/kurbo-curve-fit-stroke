//! Skeleton registration and outline-to-skeleton matching.

use kurbo::{BezPath, PathEl, Point, Vec2};

use super::extraction::extract_subpaths;
use super::{EPS, vec2_to_angle_degrees};
use crate::{InputPoint, PointType};

const SKELETON_MATCH_TOLERANCE: f64 = 2.0;

/// Stores pre-computed information about the skeleton path for angle preservation
///
/// This struct holds all necessary data to match outline points back to skeleton
/// locations. It should be created once before stroking, then reused for multiple
/// refit operations on the same skeleton.
///
/// # Fields
/// * `segments` - Pre-computed skeleton segments with geometry and width info
/// * `angles_at_points` - Incoming/outgoing angles at each on-curve skeleton point
/// * `point_types` - Point type (Corner/Smooth) at each on-curve skeleton point
#[derive(Debug, Clone)]
pub struct SkeletonInfo {
    /// Pre-computed skeleton segments for fast nearest-point queries
    /// Length = number of segments in original path
    segments: Vec<SkeletonSegment>,

    /// Incoming and outgoing angles at each on-curve skeleton point (degrees)
    /// Index corresponds to on-curve points in the original skeleton path
    /// Length = number of on-curve points
    angles_at_points: Vec<(Option<f64>, Option<f64>)>,

    /// Point types (Corner/Smooth) at each on-curve skeleton point
    /// Index corresponds to on-curve points in the original skeleton path
    /// Length = number of on-curve points
    point_types: Vec<PointType>,
}

impl SkeletonInfo {
    /// Get the point type at a specific skeleton segment endpoint
    ///
    /// The segment_index corresponds to the endpoint of that segment in the point_types array.
    /// Because point_types includes the initial MoveTo point at index 0, we need to add 1.
    ///
    /// For example:
    /// - segment 0 (first segment) ends at point 1 → point_types[1]
    /// - segment 1 ends at point 2 → point_types[2]
    /// - segment 2 ends at point 3 → point_types[3]
    ///
    /// # Arguments
    /// * `segment_index` - Index of the segment (0-based)
    ///
    /// # Returns
    /// * `Some(PointType)` - The point type at that segment endpoint
    /// * `None` - If the index is out of bounds
    fn get_point_type_at_segment(&self, segment_index: usize) -> Option<PointType> {
        // segment_index + 1 maps to the point_types array because:
        // - segment 0 ends at point 1
        // - segment 1 ends at point 2, etc.
        self.point_types.get(segment_index + 1).cloned()
    }
}

/// One segment of the skeleton path with pre-computed geometry
///
/// A segment is a single path element (LineTo, QuadTo, or CurveTo).
/// We cache geometric information to avoid recalculating during matching.
#[derive(Clone, Debug)]
struct SkeletonSegment {
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
    /// Confident if: |distance_to_skeleton - expected_offset| <= tolerance
    ///
    /// Only confident matches have their skeleton angles applied to outline points.
    /// Non-confident matches are silently ignored (outline angles preserved).
    pub is_confident_match: bool,

    /// Expected stroke offset at the matched skeleton location
    /// Calculated by interpolating widths at parameter t
    pub expected_offset: f64,
}

/// Interpolate stroke width at a parameter along a skeleton segment
///
/// Widths are specified only at skeleton on-curve points. Between points,
/// the width is linearly interpolated:
///
/// w(t) = w_start * (1 - t) + w_end * t
fn interpolate_width_at_parameter(segment: &SkeletonSegment, t: f64) -> f64 {
    let t_clamped = t.clamp(0.0, 1.0);
    segment.start_width * (1.0 - t_clamped) + segment.end_width * t_clamped
}

/// Compute the tangent vector of a skeleton segment at a specific parameter
///
/// The tangent vector represents the direction of motion along the segment.
/// For curve fitting, we convert this to an angle via atan2.
///
/// Returns a tangent vector (may be zero-length for degenerate segments).
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

            2.0 * ((1.0 - t) * (cp - p0) + t * (p2 - cp))
        }
        PathEl::CurveTo(cp1, cp2, p3) => {
            // Cubic Bézier derivative
            // B'(t) = 3(1-t)²(cp1-p0) + 6(1-t)t(cp2-cp1) + 3t²(p3-cp2)
            let t2 = t * t;
            let one_minus_t = 1.0 - t;
            let one_minus_t2 = one_minus_t * one_minus_t;

            3.0 * (one_minus_t2 * (cp1 - p0)
                + 2.0 * one_minus_t * t * (cp2 - cp1)
                + t2 * (p3 - cp2))
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
        PathEl::QuadTo(_cp, _p2) | PathEl::CurveTo(_cp, _, _p2) => {
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
///
/// 2. **At skeleton end point** (t ≈ 1.0):
///    - Use the incoming angle of the skeleton point
///
/// 3. **Between skeleton points** (0.0 < t < 1.0):
///    - Calculate the segment tangent at parameter t
///    - Convert tangent vector to angle: atan2(tan.y, tan.x)
///
/// # Returns
///
/// * `Some(angle_degrees)` - Successfully computed angle, normalized to [-180, 180]
/// * `None` - Unable to compute (degenerate tangent, invalid index, etc.)
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
    vec2_to_angle_degrees(tangent)
}

/// Register a skeleton path for angle preservation during stroke refitting
///
/// Call this BEFORE stroking to prepare the skeleton for later use in
/// `refit_stroke(stroke_path, Some(&skeleton_info), config)`. This function pre-computes all necessary
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
/// )?;
///
/// // Now stroke and refit using this skeleton_info...
/// let outline = stroker.stroke(&skeleton, &widths, &style);
/// let config = StrokeRefitterConfig::new();
/// let refitted = refit_stroke(&outline, Some(&skeleton_info), &config)?;
/// ```
pub fn register_skeleton_for_preservation(
    skeleton_path: &BezPath,
    skeleton_angles: &[InputPoint],
    widths: &[f64],
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

    // Extract angles from InputPoints, or compute them from skeleton curve if None
    let mut angles_at_points: Vec<(Option<f64>, Option<f64>)> = Vec::new();

    // Extract subpaths to compute tangents if needed
    let subpaths = extract_subpaths(skeleton_path);

    for (angle_idx, skeleton_point) in skeleton_angles.iter().enumerate() {
        let mut incoming = skeleton_point.incoming_angle;
        let mut outgoing = skeleton_point.outgoing_angle;

        // If angles are not provided, compute them from the skeleton curve
        if (incoming.is_none() || outgoing.is_none())
            && let Some(subpath) = subpaths.first()
        {
            // Compute incoming angle at this point
            if incoming.is_none() && angle_idx > 0 && angle_idx < subpath.segments.len() {
                let segment = &subpath.segments[angle_idx - 1];
                if let Some(tangent) = segment.end_tangent {
                    incoming = vec2_to_angle_degrees(tangent);
                }
            }

            // Compute outgoing angle at this point
            if outgoing.is_none() && angle_idx < subpath.segments.len() {
                let segment = &subpath.segments[angle_idx];
                if let Some(tangent) = segment.start_tangent {
                    outgoing = vec2_to_angle_degrees(tangent);
                }
            }
        }

        angles_at_points.push((incoming, outgoing));
    }

    // Extract point types from InputPoints
    let point_types: Vec<PointType> = skeleton_angles
        .iter()
        .map(|inp| inp.point_type.clone())
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
            PathEl::LineTo(p) | PathEl::QuadTo(_, p) | PathEl::CurveTo(_, _, p) => {
                if point_index >= widths.len() {
                    break;
                }

                let start_width = if point_index > 0 {
                    widths[point_index - 1]
                } else {
                    widths[0]
                };
                let end_width = widths[point_index];

                segments.push(SkeletonSegment {
                    start_point: current_pos,
                    end_point: p,
                    start_width,
                    end_width,
                    element: el,
                });

                current_pos = p;
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
        point_types,
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
/// 6. IF |actual_distance - expected_offset| <= tolerance:
///    - Match is CONFIDENT, extract skeleton angle
/// - ELSE: Match is NOT CONFIDENT, return None for this point
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

/// Detect Corner points that should actually be Smooth based on skeleton
///
/// This function identifies points that were classified as Corners during outline extraction
/// but match to Smooth points in the skeleton. These are likely false positives caused by
/// stroking artifacts or numerical precision issues, and should be corrected to Smooth.
///
/// # Arguments
///
/// * `outline_points` - The extracted outline points
/// * `skeleton_info` - Pre-computed skeleton information for matching
///
/// # Returns
///
/// Vector of indices of Corner points that have confident Smooth matches in skeleton
pub(super) fn detect_misclassified_corners(
    outline_points: &[InputPoint],
    skeleton_info: &SkeletonInfo,
) -> Vec<usize> {
    let mut misclassified = Vec::new();

    // Match all outline points to skeleton
    let skeleton_matches =
        match_outline_points_to_skeleton(outline_points, skeleton_info, SKELETON_MATCH_TOLERANCE);

    for (i, point) in outline_points.iter().enumerate() {
        // Only examine Corner points
        if !matches!(point.point_type, PointType::Corner) {
            continue;
        }

        // Check if this Corner has a confident Smooth match in skeleton
        if let Some(ref match_info) = skeleton_matches[i]
            && match_info.is_confident_match
        {
            // Check what type the skeleton point is
            if let Some(skeleton_type) =
                skeleton_info.get_point_type_at_segment(match_info.segment_index)
            {
                // If skeleton is Smooth, this Corner is likely a false positive
                if matches!(skeleton_type, PointType::Smooth) {
                    misclassified.push(i);
                }
            }
        }
    }

    misclassified
}

/// Apply skeleton angle corrections to specific failing points
///
/// # Returns
///
/// * `Ok((corrected_count, unmatched_count))` - Number of corrections applied and unmatched failures
/// * `Err(String)` - If matching fails with an error
pub(super) fn apply_skeleton_correction_to_failures(
    outline_points: &mut [InputPoint],
    failing_indices: &[usize],
    skeleton_info: &SkeletonInfo,
) -> Result<(usize, usize), String> {
    let mut corrected_count = 0;
    let mut unmatched_count = 0;

    // Match all outline points back to skeleton locations
    let skeleton_matches =
        match_outline_points_to_skeleton(outline_points, skeleton_info, SKELETON_MATCH_TOLERANCE);

    for &idx in failing_indices {
        if idx >= outline_points.len() {
            continue;
        }

        // Check if we have a confident match for this point
        if let Some(ref match_info) = skeleton_matches[idx]
            && match_info.is_confident_match
        {
            // Extract skeleton angle from the match
            if let Some(skeleton_angle) = match_info.skeleton_angle {
                // Apply skeleton angle to both incoming and outgoing
                outline_points[idx].incoming_angle = Some(skeleton_angle);
                outline_points[idx].outgoing_angle = Some(skeleton_angle);

                // Override point type with skeleton's type
                if let Some(skeleton_type) =
                    skeleton_info.get_point_type_at_segment(match_info.segment_index)
                {
                    outline_points[idx].point_type = skeleton_type;
                }

                corrected_count += 1;
                continue;
            }
        }

        // No confident match found for this failure
        unmatched_count += 1;
    }

    Ok((corrected_count, unmatched_count))
}
