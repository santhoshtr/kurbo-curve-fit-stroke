//! Stroke refitting module
//!
//! This module provides functionality to refit stroked path outlines by extracting
//! on-curve points with their incoming/outgoing tangent angles and running the
//! curve fitting algorithm to produce cleaner, optimized curves.
//!
//! Organization:
//! - [`extraction`]: subpath and tangent extraction from a `BezPath`
//! - [`smoothing`]: G1 continuity smoothing and validation
//! - [`matching`]: skeleton registration and outline-to-skeleton matching

mod extraction;
mod matching;
mod smoothing;

pub use matching::{OutlineSkeletonMatch, SkeletonInfo, register_skeleton_for_preservation};

use kurbo::{BezPath, Vec2};

use crate::fit_curve;
use extraction::{extract_subpaths, subpath_to_input_points};
use matching::{apply_skeleton_correction_to_failures, detect_misclassified_corners};
use smoothing::{
    detect_g1_failures, g1_smooth, validate_combined_path_continuity, validate_g1_smooth,
};

const DEDUP_EPSILON: f64 = 1e-6;
const EPS: f64 = 1e-12;

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for stroke refitting behavior
///
/// Controls how outline points are classified as corners vs smooth points
/// and how G1 continuity is enforced.
///
/// # Fields
///
/// * `corner_threshold_degrees` - Angle difference threshold for corner detection
///   - If incoming and outgoing angles differ by more than this, point is a corner
///   - Default: 15.0° (recommended for variable-width stroking)
///   - Lower values (10.0°) create more corners, higher values create smoother curves
///
/// * `g1_smooth_threshold_degrees` - Angle difference threshold for G1 smoothing
///   - If a Smooth point has angle difference within this threshold, average the angles
///   - Default: 15.0° (should match corner_threshold for consistency)
///   - This enforces perfect G1 continuity on the fitted curve
///
/// # Recommended Values
///
/// - **Variable-width stroking**: 15.0° / 15.0° (geometric distortion creates 12-13° differences)
/// - **Constant-width stroking**: 10.0° / 10.0° (tighter tolerance, less geometric noise)
/// - **Aggressive smoothing**: 20.0° / 20.0° (fewer corners, smoother curves)
/// - **Precise corners**: 5.0° / 5.0° (preserve intentional corners)
#[derive(Debug, Clone, Copy)]
pub struct StrokeRefitterConfig {
    /// Angle threshold in degrees for corner detection
    pub corner_threshold_degrees: f64,
    /// Angle threshold in degrees for G1 smoothing
    pub g1_smooth_threshold_degrees: f64,
}

impl StrokeRefitterConfig {
    /// Default configuration optimized for variable-width stroking
    pub fn new() -> Self {
        Self {
            corner_threshold_degrees: 15.0,
            g1_smooth_threshold_degrees: 15.0,
        }
    }

    /// Configuration for constant-width stroking (stricter corner detection)
    pub fn constant_width() -> Self {
        Self {
            corner_threshold_degrees: 10.0,
            g1_smooth_threshold_degrees: 10.0,
        }
    }

    /// Configuration for aggressive smoothing (fewer corners)
    pub fn aggressive_smooth() -> Self {
        Self {
            corner_threshold_degrees: 20.0,
            g1_smooth_threshold_degrees: 20.0,
        }
    }

    /// Configuration for precise corner preservation
    pub fn precise_corners() -> Self {
        Self {
            corner_threshold_degrees: 5.0,
            g1_smooth_threshold_degrees: 5.0,
        }
    }

    /// Custom configuration with specified thresholds
    pub fn custom(corner: f64, g1_smooth: f64) -> Self {
        Self {
            corner_threshold_degrees: corner,
            g1_smooth_threshold_degrees: g1_smooth,
        }
    }
}

impl Default for StrokeRefitterConfig {
    fn default() -> Self {
        Self::new()
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
/// ## Skeleton-Aware Refitting
///
/// This function uses a hybrid approach when skeleton information is provided:
///
/// **Pipeline with skeleton:**
/// 1. Extract on-curve points from stroke outline with outline-derived angles
/// 2. Classify point types (Corner/Smooth) based on angle differences
/// 3. Apply G1 smoothing to enforce continuity (averaging angles if close)
/// 4. Detect which Smooth points STILL fail G1 after smoothing
/// 5. For failing points ONLY: consult skeleton and apply angle corrections
/// 6. Override point types to match skeleton for corrected points
/// 7. Re-apply G1 smoothing to propagate corrections to neighboring points
/// 8. Validate G1 smoothing (warnings only if failures remain)
/// 9. Fit final curve using Hobby's algorithm
///
/// **Pipeline without skeleton (outline-only):**
/// 1. Extract on-curve points from stroke outline
/// 2. Apply G1 smoothing to enforce continuity
/// 3. Validate smoothing was applied correctly
/// 4. Fit the curve using Hobby's algorithm
///
/// **Key Principle:** When skeleton is available, it's used as a **correction tool**
/// for specific failures, not as the primary source. This preserves the natural
/// smoothness of outline geometry while fixing real problems.
///
/// # Arguments
///
/// * `stroke_path` - The stroked path outline to refit
/// * `skeleton_info` - Optional skeleton information for error correction
/// * `config` - Refitter configuration (thresholds, smoothing parameters)
///
/// # Returns
///
/// * `Ok(BezPath)` - The refitted curve combining all subpaths
/// * `Err(String)` - Error message if fitting fails (aborts on first failure)
///
/// # Examples
///
/// ```text
/// use kurbo::BezPath;
/// use curve_fitter::{refit_stroke, StrokeRefitterConfig};
///
/// let config = StrokeRefitterConfig::new();
///
/// // Without skeleton (outline-only refitting)
/// let refitted = refit_stroke(&stroked_path, None, &config)?;
///
/// // With skeleton (selective skeleton correction)
/// let refitted = refit_stroke(&stroked_path, Some(&skeleton_info), &config)?;
/// ```
///
/// # Implementation Notes
///
/// - G1 failure detection (when skeleton available) uses 0.5° tolerance
/// - Only points with confident skeleton matches are corrected
/// - Points with unmatched failures are left as-is (may generate warnings)
/// - Fallback behavior: warns and continues if corrections don't fix all failures
pub fn refit_stroke(
    stroke_path: &BezPath,
    skeleton_info: Option<&SkeletonInfo>,
    config: &StrokeRefitterConfig,
) -> Result<BezPath, String> {
    const G1_FAILURE_TOLERANCE_DEGREES: f64 = 0.5;

    let subpaths = extract_subpaths(stroke_path);

    if subpaths.is_empty() {
        return Ok(BezPath::new());
    }

    let mut combined_path = BezPath::new();
    let mut is_first_subpath = true;

    for subpath in subpaths {
        let mut input_points =
            subpath_to_input_points(&subpath, config.corner_threshold_degrees, DEDUP_EPSILON);

        if input_points.len() < 2 {
            continue;
        }

        // Stage 1: Apply G1 smoothing to outline-derived angles
        g1_smooth(&mut input_points, config.g1_smooth_threshold_degrees);

        // If skeleton is provided, apply selective skeleton correction
        if let Some(skeleton) = skeleton_info {
            // Stage 1b: Detect and correct misclassified Corners
            // Some Corner points might be false positives from stroking artifacts
            // If they match to Smooth skeleton points, correct them
            let misclassified_corners = detect_misclassified_corners(&input_points, skeleton);
            if !misclassified_corners.is_empty() {
                match apply_skeleton_correction_to_failures(
                    &mut input_points,
                    &misclassified_corners,
                    skeleton,
                ) {
                    Ok((corrected_count, unmatched_count)) => {
                        eprintln!(
                            "Corrected misclassified corners: {} detected, {} corrected, {} unmatched",
                            misclassified_corners.len(),
                            corrected_count,
                            unmatched_count
                        );
                    }
                    Err(e) => {
                        eprintln!("Warning: Could not correct misclassified corners: {}", e);
                    }
                }
            }

            // Stage 2: Detect failures and apply selective skeleton correction
            let failures = detect_g1_failures(&input_points, G1_FAILURE_TOLERANCE_DEGREES);
            if !failures.is_empty() {
                // Try to correct failing points using skeleton information
                match apply_skeleton_correction_to_failures(&mut input_points, &failures, skeleton)
                {
                    Ok((corrected_count, unmatched_count)) => {
                        eprintln!(
                            "Selective correction: {} failures detected, {} corrected, {} unmatched",
                            failures.len(),
                            corrected_count,
                            unmatched_count
                        );

                        // Re-apply G1 smoothing to all points after corrections
                        // This propagates skeleton-corrected angles to neighbors
                        g1_smooth(&mut input_points, config.g1_smooth_threshold_degrees);

                        // Validate again (but don't fail if it still doesn't pass)
                        match validate_g1_smooth(&input_points) {
                            Ok(()) => {
                                // Success: corrections fixed the problem
                            }
                            Err(_e) => {
                                // Fallback: warn but continue
                                eprintln!(
                                    "Warning: G1 validation still failed after skeleton correction (fallback: continuing anyway)"
                                );
                            }
                        }
                    }
                    Err(e) => {
                        // Matching error - warn but continue
                        eprintln!("Warning: Skeleton matching failed during correction: {}", e);
                    }
                }
            } else {
                // No failures detected: validate that smoothing worked correctly
                validate_g1_smooth(&input_points)?;
            }
        } else {
            // No skeleton provided: validate smoothing was applied correctly
            validate_g1_smooth(&input_points)?;
        }

        // Stage 3: Fit the curve
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

    // Validate that the combined path maintains G1 continuity
    // This checks the actual output curve for kinks
    validate_combined_path_continuity(&combined_path, config, skeleton_info.is_some())?;

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
}
