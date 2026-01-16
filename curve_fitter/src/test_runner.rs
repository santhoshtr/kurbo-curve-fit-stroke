//! Test runner for JSON-driven test cases
//!
//! This module provides the TestRunner which executes test cases defined in JSON files,
//! performing curve fitting operations and generating SVG output files.

use std::fs;
use std::path::PathBuf;

use kurbo::BezPath;

use crate::test_schema::{InputPointJson, TestCase};
use crate::var_stroke::VariableStroke;
use crate::var_stroker::VariableStroker;
use crate::{
    InputPoint, PointType, SkeletonInfo, StrokeRefitterConfig, fit_curve, refit_stroke,
    refit_stroke_with_skeleton_correction, register_skeleton_for_preservation,
};

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

        let mut intermediate_results = IntermediateResults::default();

        // Convert input points from JSON
        let input_points = self.convert_points(&test_case.input.points)?;
        let is_closed = test_case.input.is_closed;

        // Store for skeleton registration
        intermediate_results.input_points = Some(input_points.clone());
        intermediate_results.is_closed = is_closed;

        // Execute each operation in sequence
        for operation in &test_case.operations {
            match operation.as_str() {
                "fit_curve" => {
                    println!("  → fit_curve");
                    let fitted_path = fit_curve(input_points.clone(), is_closed)?;
                    intermediate_results.fitted_path = Some(fitted_path.clone());

                    // Write SVG if output configured
                    if let Some(filename) = &test_case.outputs.fitted_curve {
                        self.write_svg(&format!("{}.svg", filename), &fitted_path)?;
                        println!("    Written: outputs/{}.svg", filename);
                    }
                }
                "stroke" => {
                    println!("  → stroke");
                    let fitted_path = intermediate_results
                        .fitted_path
                        .as_ref()
                        .ok_or_else(|| "fit_curve must be executed before stroke".to_string())?;

                    let stroke_config = test_case.stroke.as_ref().ok_or_else(|| {
                        "stroke configuration required for stroke operation".to_string()
                    })?;

                    // Validate width count matches point count
                    let point_count = input_points.len();
                    if stroke_config.widths.len() != point_count {
                        return Err(format!(
                            "Width count mismatch: expected {}, got {}",
                            point_count,
                            stroke_config.widths.len()
                        ));
                    }

                    let stroker = VariableStroker::new(stroke_config.tolerance);
                    let stroke_style = VariableStroke::default();
                    let stroke_path =
                        stroker.stroke(fitted_path, &stroke_config.widths, &stroke_style);

                    intermediate_results.stroke_path = Some(stroke_path.clone());
                    intermediate_results.stroke_widths = Some(stroke_config.widths.clone());

                    // Write SVG if output configured
                    if let Some(filename) = &test_case.outputs.stroke_outline {
                        self.write_svg(&format!("{}.svg", filename), &stroke_path)?;
                        println!("    Written: outputs/{}.svg", filename);
                    }
                }
                "refit_stroke" => {
                    println!("  → refit_stroke");
                    let stroke_path = intermediate_results
                        .stroke_path
                        .as_ref()
                        .ok_or_else(|| "stroke must be executed before refit_stroke".to_string())?;

                    let config = StrokeRefitterConfig::new();
                    let refitted_path = refit_stroke(stroke_path, &config)?;

                    intermediate_results.refitted_path = Some(refitted_path.clone());

                    // Write SVG if output configured
                    if let Some(filename) = &test_case.outputs.refitted_stroke {
                        self.write_svg(&format!("{}.svg", filename), &refitted_path)?;
                        println!("    Written: outputs/{}.svg", filename);
                    }
                }
                "register_skeleton" => {
                    println!("  → register_skeleton");
                    let fitted_path =
                        intermediate_results.fitted_path.as_ref().ok_or_else(|| {
                            "fit_curve must be executed before register_skeleton".to_string()
                        })?;

                    let skeleton_angles = intermediate_results
                        .input_points
                        .as_ref()
                        .ok_or_else(|| "input_points not available".to_string())?;

                    let widths = intermediate_results.stroke_widths.as_ref().ok_or_else(|| {
                        "stroke must be executed before register_skeleton".to_string()
                    })?;

                    let skeleton_info = register_skeleton_for_preservation(
                        fitted_path,
                        skeleton_angles,
                        widths,
                        is_closed,
                    )?;

                    intermediate_results.skeleton_info = Some(skeleton_info);
                    println!("    Skeleton registered for angle preservation");
                }
                "refit_with_skeleton_correction" => {
                    println!("  → refit_with_skeleton_correction");
                    let stroke_path =
                        intermediate_results.stroke_path.as_ref().ok_or_else(|| {
                            "stroke must be executed before refit_with_skeleton_correction"
                                .to_string()
                        })?;

                    let skeleton_info =
                        intermediate_results.skeleton_info.as_ref().ok_or_else(|| {
                            "register_skeleton must be executed before refit_with_skeleton_correction"
                                .to_string()
                        })?;

                    let config = StrokeRefitterConfig::new();
                    let skeleton_corrected =
                        refit_stroke_with_skeleton_correction(stroke_path, skeleton_info, &config)?;

                    intermediate_results.skeleton_corrected = Some(skeleton_corrected.clone());

                    // Write SVG if output configured
                    if let Some(filename) = &test_case.outputs.skeleton_corrected {
                        self.write_svg(&format!("{}.svg", filename), &skeleton_corrected)?;
                        println!("    Written: outputs/{}.svg", filename);
                    }
                }
                _ => return Err(format!("Unknown operation: {}", operation)),
            }
        }

        println!("✓ Test completed successfully");
        Ok(())
    }

    /// Convert JSON input points to library InputPoint format
    fn convert_points(&self, json_points: &[InputPointJson]) -> Result<Vec<InputPoint>, String> {
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

    /// Write a BezPath as an SVG file
    fn write_svg(&self, filename: &str, path: &BezPath) -> Result<(), String> {
        let svg_content = bezpath_to_svg(path);
        let filepath = self.output_dir.join(filename);

        fs::write(&filepath, svg_content).map_err(|e| format!("Failed to write SVG: {}", e))?;

        Ok(())
    }
}

/// Intermediate results stored during test execution
#[derive(Debug, Default)]
struct IntermediateResults {
    input_points: Option<Vec<InputPoint>>,
    is_closed: bool,
    fitted_path: Option<BezPath>,
    stroke_path: Option<BezPath>,
    stroke_widths: Option<Vec<f64>>,
    refitted_path: Option<BezPath>,
    skeleton_info: Option<SkeletonInfo>,
    skeleton_corrected: Option<BezPath>,
}

/// Convert a BezPath to SVG string
fn bezpath_to_svg(path: &BezPath) -> String {
    let mut svg = String::new();
    svg.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    svg.push_str("<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 1000 1000\" width=\"1000\" height=\"1000\">\n");

    // Add background
    svg.push_str("  <rect width=\"1000\" height=\"1000\" fill=\"white\"/>\n");

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

    svg.push_str("\" fill=\"none\" stroke=\"black\" stroke-width=\"1\"/>\n");
    svg.push_str("</svg>\n");

    svg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_conversion() {
        let runner = TestRunner::new(false, false, PathBuf::from("outputs"));

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

        let result = runner.convert_points(&json_points);
        assert!(result.is_ok());

        let points = result.unwrap();
        assert_eq!(points.len(), 2);
        assert_eq!(points[0].x, 50.0);
        assert_eq!(points[0].outgoing_angle, Some(45.0));
        assert_eq!(points[1].incoming_angle, Some(90.0));
    }
}
