//! WebAssembly bindings for the hobby curve library

use crate::{hobby, point::Point, BezierSegment};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use wasm_bindgen::prelude::*;

// Use wee_alloc as the global allocator for smaller WASM size
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

// Enable console.log! from wasm
#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

macro_rules! console_log {
    ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
}

/// A point that can be passed between JavaScript and WebAssembly
#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct WebPoint {
    x: f64,
    y: f64,
}

#[wasm_bindgen]
impl WebPoint {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> WebPoint {
        WebPoint { x, y }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.x
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.y
    }

    #[wasm_bindgen(setter)]
    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    #[wasm_bindgen(setter)]
    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }
}

impl From<Point> for WebPoint {
    fn from(p: Point) -> Self {
        WebPoint { x: p.x, y: p.y }
    }
}

impl From<WebPoint> for Point {
    fn from(p: WebPoint) -> Self {
        Point::new(p.x, p.y)
    }
}

/// A Bézier curve segment that can be returned to JavaScript
#[wasm_bindgen]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WebBezierSegment {
    start_x: f64,
    start_y: f64,
    cp1_x: f64,
    cp1_y: f64,
    cp2_x: f64,
    cp2_y: f64,
    end_x: f64,
    end_y: f64,
}

#[wasm_bindgen]
impl WebBezierSegment {
    #[wasm_bindgen(getter)]
    pub fn start_x(&self) -> f64 {
        self.start_x
    }

    #[wasm_bindgen(getter)]
    pub fn start_y(&self) -> f64 {
        self.start_y
    }

    #[wasm_bindgen(getter)]
    pub fn cp1_x(&self) -> f64 {
        self.cp1_x
    }

    #[wasm_bindgen(getter)]
    pub fn cp1_y(&self) -> f64 {
        self.cp1_y
    }

    #[wasm_bindgen(getter)]
    pub fn cp2_x(&self) -> f64 {
        self.cp2_x
    }

    #[wasm_bindgen(getter)]
    pub fn cp2_y(&self) -> f64 {
        self.cp2_y
    }

    #[wasm_bindgen(getter)]
    pub fn end_x(&self) -> f64 {
        self.end_x
    }

    #[wasm_bindgen(getter)]
    pub fn end_y(&self) -> f64 {
        self.end_y
    }

    /// Convert to SVG path segment string
    #[wasm_bindgen]
    pub fn to_svg_path(&self) -> String {
        format!(
            "C {:.2},{:.2} {:.2},{:.2} {:.2},{:.2}",
            self.cp1_x, self.cp1_y, self.cp2_x, self.cp2_y, self.end_x, self.end_y
        )
    }
}

impl From<BezierSegment> for WebBezierSegment {
    fn from(segment: BezierSegment) -> Self {
        WebBezierSegment {
            start_x: segment.0.x,
            start_y: segment.0.y,
            cp1_x: segment.1.x,
            cp1_y: segment.1.y,
            cp2_x: segment.2.x,
            cp2_y: segment.2.y,
            end_x: segment.3.x,
            end_y: segment.3.y,
        }
    }
}

/// Options for configuring the hobby algorithm from JavaScript
#[wasm_bindgen]
#[derive(Clone, Debug, Default)]
pub struct HobbyOptions {
    cyclic: bool,
    tensions: Option<Vec<f64>>,
    entry_angles: HashMap<usize, f64>,
    exit_angles: HashMap<usize, f64>,
}

#[wasm_bindgen]
impl HobbyOptions {
    #[wasm_bindgen(constructor)]
    pub fn new() -> HobbyOptions {
        HobbyOptions::default()
    }

    #[wasm_bindgen]
    pub fn set_cyclic(&mut self, cyclic: bool) {
        self.cyclic = cyclic;
    }

    #[wasm_bindgen]
    pub fn set_tensions(&mut self, tensions: Vec<f64>) {
        self.tensions = Some(tensions);
    }

    #[wasm_bindgen]
    pub fn add_entry_angle(&mut self, point_index: usize, angle: f64) {
        self.entry_angles.insert(point_index, angle);
    }

    #[wasm_bindgen]
    pub fn add_exit_angle(&mut self, point_index: usize, angle: f64) {
        self.exit_angles.insert(point_index, angle);
    }

    #[wasm_bindgen]
    pub fn clear_angles(&mut self) {
        self.entry_angles.clear();
        self.exit_angles.clear();
    }
}

/// Main function to generate hobby curves from JavaScript
///
/// # Arguments
/// * `points` - Array of WebPoint objects
/// * `options` - Optional HobbyOptions object (can be null)
///
/// # Returns
/// Array of WebBezierSegment objects representing the curve
#[wasm_bindgen]
pub fn hobby_curve(points: Vec<WebPoint>, options: Option<HobbyOptions>) -> Vec<WebBezierSegment> {
    console_log!("hobby_curve called with {} points", points.len());

    if points.len() < 2 {
        console_log!("Not enough points for curve generation");
        return vec![];
    }

    // Convert WebPoints to internal Points
    let internal_points: Vec<Point> = points.iter().map(|p| (*p).into()).collect();

    let opts = options.unwrap_or_default();

    // Prepare optional parameters
    let tensions_ref = opts.tensions.as_deref();
    let entry_angles_ref = if opts.entry_angles.is_empty() {
        None
    } else {
        Some(&opts.entry_angles)
    };
    let exit_angles_ref = if opts.exit_angles.is_empty() {
        None
    } else {
        Some(&opts.exit_angles)
    };

    console_log!(
        "Calling hobby with cyclic={}, {} tensions, {} entry angles, {} exit angles",
        opts.cyclic,
        opts.tensions.as_ref().map_or(0, |t| t.len()),
        opts.entry_angles.len(),
        opts.exit_angles.len()
    );

    // Call the main hobby function
    let segments = hobby(
        &internal_points,
        tensions_ref,
        opts.cyclic,
        entry_angles_ref,
        exit_angles_ref,
    );

    console_log!("Generated {} segments", segments.len());

    // Convert to web-friendly format
    segments.into_iter().map(|seg| seg.into()).collect()
}

/// Simple convenience function for basic curve generation
/// Takes a flat array of coordinates [x1, y1, x2, y2, ...]
#[wasm_bindgen]
pub fn hobby_curve_simple(coords: Vec<f64>) -> Vec<WebBezierSegment> {
    if coords.len() % 2 != 0 || coords.len() < 4 {
        console_log!("Invalid coordinates array length: {}", coords.len());
        return vec![];
    }

    let points: Vec<WebPoint> = coords
        .chunks_exact(2)
        .map(|chunk| WebPoint::new(chunk[0], chunk[1]))
        .collect();

    hobby_curve(points, None)
}

/// Generate an SVG path string from points
#[wasm_bindgen]
pub fn hobby_to_svg_path(points: Vec<WebPoint>, options: Option<HobbyOptions>) -> String {
    let segments = hobby_curve(points, options);

    if segments.is_empty() {
        return String::new();
    }

    let mut path = format!("M {:.2},{:.2}", segments[0].start_x, segments[0].start_y);

    for segment in segments {
        path.push(' ');
        path.push_str(&segment.to_svg_path());
    }

    path
}

/// Initialize the WASM module (call this first from JavaScript)
#[wasm_bindgen(start)]
pub fn main() {
    console_log!("Hobby WASM module initialized");

    // Set up panic hook for better error messages
    #[cfg(feature = "console_error_panic_hook")]
    console_error_panic_hook::set_once();
}

/// Get version information
#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

// Utility functions for debugging and testing

/// Create a simple test curve for validation
#[wasm_bindgen]
pub fn create_test_curve() -> Vec<WebBezierSegment> {
    let points = vec![
        WebPoint::new(0.0, 0.0),
        WebPoint::new(100.0, 100.0),
        WebPoint::new(200.0, 0.0),
    ];

    hobby_curve(points, None)
}

/// Validate that a point array has the correct format
#[wasm_bindgen]
pub fn validate_points(points: Vec<WebPoint>) -> bool {
    if points.len() < 2 {
        return false;
    }

    // Check for NaN or infinite values
    for point in points {
        if !point.x.is_finite() || !point.y.is_finite() {
            return false;
        }
    }

    true
}
