//! Test runner for JSON-driven test cases
//!
//! This module provides the TestRunner which executes test cases defined in JSON files,
//! performing curve fitting operations and generating SVG output files.

use std::fs;
use std::path::PathBuf;

use kurbo::BezPath;

use crate::test_schema::{InputPointJson, TestCase};
use curve_fitter::var_stroke::VariableStroke;
use curve_fitter::var_stroker::VariableStroker;
use curve_fitter::{
    InputPoint, PointType, RefitDiagnostics, SkeletonInfo, StrokeRefitterConfig, fit_curve,
    refit_stroke, register_skeleton_for_preservation,
};

/// Result of one operation of a test case
pub struct StageOutput {
    pub operation: String,
    /// Path produced by the operation (None for register_skeleton)
    pub path: Option<BezPath>,
    /// Diagnostics for refit operations
    pub diagnostics: Option<RefitDiagnostics>,
}

/// Execute the operations of a test case and return each stage's output
pub fn execute_operations(test_case: &TestCase) -> Result<Vec<StageOutput>, String> {
    let input_points = convert_points(&test_case.input.points)?;
    let is_closed = test_case.input.is_closed;

    let mut fitted_path: Option<BezPath> = None;
    let mut stroke_path: Option<BezPath> = None;
    let mut stroke_widths: Option<Vec<f64>> = None;
    let mut skeleton_info: Option<SkeletonInfo> = None;
    let mut stages = Vec::new();

    for operation in &test_case.operations {
        let mut stage = StageOutput {
            operation: operation.clone(),
            path: None,
            diagnostics: None,
        };

        match operation.as_str() {
            "fit_curve" => {
                let path = fit_curve(input_points.clone(), is_closed)?;
                fitted_path = Some(path.clone());
                stage.path = Some(path);
            }
            "stroke" => {
                let fitted = fitted_path
                    .as_ref()
                    .ok_or_else(|| "fit_curve must be executed before stroke".to_string())?;

                let stroke_config = test_case.stroke.as_ref().ok_or_else(|| {
                    "stroke configuration required for stroke operation".to_string()
                })?;

                // Validate width count matches point count
                if stroke_config.widths.len() != input_points.len() {
                    return Err(format!(
                        "Width count mismatch: expected {}, got {}",
                        input_points.len(),
                        stroke_config.widths.len()
                    ));
                }

                let stroker = VariableStroker::new(stroke_config.tolerance);
                let path = stroker.stroke(fitted, &stroke_config.widths, &VariableStroke::default())?;

                stroke_widths = Some(stroke_config.widths.clone());
                stroke_path = Some(path.clone());
                stage.path = Some(path);
            }
            "refit_stroke" => {
                let stroked = stroke_path
                    .as_ref()
                    .ok_or_else(|| "stroke must be executed before refit_stroke".to_string())?;

                let (path, diagnostics) =
                    refit_stroke(stroked, None, &StrokeRefitterConfig::new())?;
                stage.path = Some(path);
                stage.diagnostics = Some(diagnostics);
            }
            "register_skeleton" => {
                let fitted = fitted_path.as_ref().ok_or_else(|| {
                    "fit_curve must be executed before register_skeleton".to_string()
                })?;

                let widths = stroke_widths.as_ref().ok_or_else(|| {
                    "stroke must be executed before register_skeleton".to_string()
                })?;

                skeleton_info = Some(register_skeleton_for_preservation(
                    fitted,
                    &input_points,
                    widths,
                )?);
            }
            "refit_stroke_with_skeleton" => {
                let stroked = stroke_path.as_ref().ok_or_else(|| {
                    "stroke must be executed before refit_stroke_with_skeleton".to_string()
                })?;

                let skeleton = skeleton_info.as_ref().ok_or_else(|| {
                    "register_skeleton must be executed before refit_stroke_with_skeleton"
                        .to_string()
                })?;

                let (path, diagnostics) =
                    refit_stroke(stroked, Some(skeleton), &StrokeRefitterConfig::new())?;
                stage.path = Some(path);
                stage.diagnostics = Some(diagnostics);
            }
            _ => return Err(format!("Unknown operation: {}", operation)),
        }

        stages.push(stage);
    }

    Ok(stages)
}

/// Output filename configured for an operation, if any
fn output_filename<'a>(test_case: &'a TestCase, operation: &str) -> Option<&'a String> {
    match operation {
        "fit_curve" => test_case.outputs.fitted_curve.as_ref(),
        "stroke" => test_case.outputs.stroke_outline.as_ref(),
        "refit_stroke" => test_case.outputs.refitted_stroke.as_ref(),
        "refit_stroke_with_skeleton" => test_case.outputs.skeleton_corrected.as_ref(),
        _ => None,
    }
}

/// Convert JSON input points to library InputPoint format
fn convert_points(json_points: &[InputPointJson]) -> Result<Vec<InputPoint>, String> {
    json_points
        .iter()
        .map(|p| {
            let point_type = match p.point_type.as_str() {
                "Smooth" => PointType::Smooth,
                "Corner" => PointType::Corner,
                "LineToCurve" => PointType::LineToCurve,
                "CurveToLine" => PointType::CurveToLine,
                _ => return Err(format!("Unknown point type: {}", p.point_type)),
            };

            Ok(InputPoint {
                x: p.x,
                y: p.y,
                point_type,
                incoming_angle: p.incoming_angle,
                outgoing_angle: p.outgoing_angle,
            })
        })
        .collect()
}

/// Test execution engine
pub struct TestRunner {
    /// Whether to print detailed validation output
    #[allow(dead_code)]
    verbose: bool,

    /// Whether to skip G1 continuity validation
    #[allow(dead_code)]
    skip_validation: bool,

    /// Directory for output SVG files
    output_dir: PathBuf,
}

impl TestRunner {
    /// Create a new test runner
    pub fn new(verbose: bool, skip_validation: bool, output_dir: PathBuf) -> Self {
        Self {
            verbose,
            skip_validation,
            output_dir,
        }
    }

    /// Run a single test case
    pub fn run_test(&self, test_case: &TestCase) -> Result<(), String> {
        println!("\n=== Running test: {} ===", test_case.name);
        println!("Description: {}", test_case.description);

        let stages = execute_operations(test_case)?;

        for stage in &stages {
            println!("  → {}", stage.operation);

            if let Some(diagnostics) = &stage.diagnostics {
                print_diagnostics(diagnostics);
            }

            if stage.operation == "register_skeleton" {
                println!("    Skeleton registered for angle preservation");
            }

            if let Some(path) = &stage.path
                && let Some(filename) = output_filename(test_case, &stage.operation)
            {
                self.write_svg(&format!("{}.svg", filename), path)?;
                println!("    Written: outputs/{}.svg", filename);
            }
        }

        println!("✓ Test completed successfully");
        Ok(())
    }

    /// Write a BezPath as an SVG file
    fn write_svg(&self, filename: &str, path: &BezPath) -> Result<(), String> {
        let svg_content = bezpath_to_svg(path);
        let filepath = self.output_dir.join(filename);

        fs::write(&filepath, svg_content).map_err(|e| format!("Failed to write SVG: {}", e))?;

        Ok(())
    }
}

/// Print refit diagnostics collected by the library
fn print_diagnostics(diagnostics: &RefitDiagnostics) {
    if diagnostics.misclassified_corners > 0 {
        println!(
            "    Misclassified corners: {} detected, {} corrected",
            diagnostics.misclassified_corners, diagnostics.misclassified_corners_corrected
        );
    }
    if diagnostics.g1_failures > 0 {
        println!(
            "    G1 failures: {} detected, {} corrected",
            diagnostics.g1_failures, diagnostics.g1_failures_corrected
        );
    }
    for warning in &diagnostics.warnings {
        println!("    Warning: {}", warning);
    }
}

/// Convert a BezPath to SVG string
fn bezpath_to_svg(path: &BezPath) -> String {
    let mut svg = String::new();
    svg.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    svg.push_str("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 400 200\" width=\"400\" height=\"200\">\n");

    // Add path
    svg.push_str("  <path d=\"");

    for el in path.iter() {
        match el {
            kurbo::PathEl::MoveTo(p) => {
                svg.push_str(&format!("M{},{}", p.x, p.y));
            }
            kurbo::PathEl::LineTo(p) => {
                svg.push_str(&format!("L{},{}", p.x, p.y));
            }
            kurbo::PathEl::QuadTo(p1, p2) => {
                svg.push_str(&format!("Q{},{} {},{}", p1.x, p1.y, p2.x, p2.y));
            }
            kurbo::PathEl::CurveTo(p1, p2, p3) => {
                svg.push_str(&format!(
                    "C{},{} {},{} {},{}",
                    p1.x, p1.y, p2.x, p2.y, p3.x, p3.y
                ));
            }
            kurbo::PathEl::ClosePath => {
                svg.push('Z');
            }
        }
    }

    svg.push_str("\" fill=\"#AFDBF5\" stroke=\"orangered\" stroke-width=\"1\"/>\n");
    svg.push_str("</svg>\n");

    svg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_conversion() {
        let json_points = vec![
            InputPointJson {
                x: 50.0,
                y: 100.0,
                point_type: "Smooth".to_string(),
                incoming_angle: None,
                outgoing_angle: Some(45.0),
            },
            InputPointJson {
                x: 150.0,
                y: 50.0,
                point_type: "Corner".to_string(),
                incoming_angle: Some(90.0),
                outgoing_angle: Some(180.0),
            },
        ];

        let result = convert_points(&json_points);
        assert!(result.is_ok());

        let points = result.unwrap();
        assert_eq!(points.len(), 2);
        assert_eq!(points[0].x, 50.0);
        assert_eq!(points[0].outgoing_angle, Some(45.0));
        assert_eq!(points[1].incoming_angle, Some(90.0));
    }
}
