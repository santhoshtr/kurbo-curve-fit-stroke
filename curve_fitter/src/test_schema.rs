//! JSON schema definitions for test cases
//!
//! This module defines serializable types for test case JSON files.
//! Test cases describe curve fitting operations and their expected outputs.

use serde::{Deserialize, Serialize};

/// A complete test case specification
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct TestCase {
    /// Name of the test case (used for file naming and identification)
    pub name: String,

    /// Human-readable description of what this test does
    pub description: String,

    /// Type of test (fit_curve, fit_and_stroke, stroke_only, etc.)
    pub test_type: String,

    /// Input curve data
    pub input: TestInput,

    /// Stroke configuration (if test involves stroking)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stroke: Option<StrokeConfig>,

    /// Sequence of operations to perform
    pub operations: Vec<String>,

    /// Output file names for each stage
    pub outputs: OutputConfig,
}

/// Input curve specification using InputPoints
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct TestInput {
    /// Array of control points
    pub points: Vec<InputPointJson>,

    /// Whether the path is closed
    pub is_closed: bool,
}

/// A single control point in JSON format
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct InputPointJson {
    /// X coordinate
    pub x: f64,

    /// Y coordinate
    pub y: f64,

    /// Point type: "Smooth" or "Corner"
    pub point_type: String,

    /// Optional incoming tangent angle in degrees
    #[serde(skip_serializing_if = "Option::is_none")]
    pub incoming_angle: Option<f64>,

    /// Optional outgoing tangent angle in degrees
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outgoing_angle: Option<f64>,
}

/// Stroke configuration
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct StrokeConfig {
    /// Width at each on-curve point
    pub widths: Vec<f64>,

    /// Tolerance for stroke computation
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
}

/// Output file configuration
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct OutputConfig {
    /// Output filename for fitted curve (without extension)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fitted_curve: Option<String>,

    /// Output filename for stroke outline (without extension)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stroke_outline: Option<String>,

    /// Output filename for refitted stroke (without extension)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refitted_stroke: Option<String>,

    /// Output filename for skeleton-preserved result (without extension)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skeleton_preserved: Option<String>,
}

fn default_tolerance() -> f64 {
    0.1
}

impl TestCase {
    /// Load a test case from a JSON file
    pub fn from_file(path: &std::path::Path) -> Result<Self, Box<dyn std::error::Error>> {
        let content = std::fs::read_to_string(path)?;
        let test_case = serde_json::from_str(&content)?;
        Ok(test_case)
    }

    /// Save a test case to a JSON file
    pub fn to_file(&self, path: &std::path::Path) -> Result<(), Box<dyn std::error::Error>> {
        let content = serde_json::to_string_pretty(self)?;
        std::fs::write(path, content)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deserialize_simple_test_case() {
        let json = r#"{
            "name": "wave-simple",
            "description": "Simple wave curve",
            "test_type": "fit_and_stroke",
            "input": {
                "points": [
                    {"x": 50.0, "y": 100.0, "point_type": "Smooth"}
                ],
                "is_closed": false
            },
            "stroke": {
                "widths": [10.0],
                "tolerance": 0.1
            },
            "operations": ["fit_curve", "stroke", "refit_stroke"],
            "outputs": {
                "fitted_curve": "fitted",
                "stroke_outline": "outline",
                "refitted_stroke": "refitted"
            }
        }"#;

        let test_case: TestCase = serde_json::from_str(json).unwrap();
        assert_eq!(test_case.name, "wave-simple");
        assert_eq!(test_case.input.points.len(), 1);
    }
}
