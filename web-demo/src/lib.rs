use curve_fitter::{
    InputPoint, PointType, StrokeRefitterConfig, refit_stroke,
    refit_stroke_with_skeleton_correction, register_skeleton_for_preservation,
    var_stroke::VariableStroke, var_stroker::VariableStroker,
};
use kurbo::{BezPath, Cap, Join, Point, Stroke, StrokeOpts, stroke};
use wasm_bindgen::prelude::*;

#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);

    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

macro_rules! console_log {
    ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
}

#[wasm_bindgen]
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[wasm_bindgen]
#[derive(Clone, Copy)]
pub enum WebPointType {
    Smooth = 0,
    Corner = 1,
}

#[wasm_bindgen]
pub struct WebPoint {
    x: f64,
    y: f64,
    point_type: WebPointType,
    incoming_angle: Option<f64>, // in degrees
    outgoing_angle: Option<f64>, // in degrees
}

#[wasm_bindgen]
impl WebPoint {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> WebPoint {
        WebPoint {
            x,
            y,
            point_type: WebPointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
        }
    }

    #[wasm_bindgen]
    pub fn new_with_type(x: f64, y: f64, point_type: WebPointType) -> WebPoint {
        WebPoint {
            x,
            y,
            point_type,
            incoming_angle: None,
            outgoing_angle: None,
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.x
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.y
    }

    #[wasm_bindgen(getter)]
    pub fn point_type(&self) -> WebPointType {
        self.point_type
    }

    #[wasm_bindgen(setter)]
    pub fn set_x(&mut self, x: f64) {
        self.x = x;
    }

    #[wasm_bindgen(setter)]
    pub fn set_y(&mut self, y: f64) {
        self.y = y;
    }

    #[wasm_bindgen(setter)]
    pub fn set_point_type(&mut self, point_type: WebPointType) {
        self.point_type = point_type;
    }

    #[wasm_bindgen]
    pub fn incoming_angle(&self) -> Option<f64> {
        self.incoming_angle
    }

    #[wasm_bindgen]
    pub fn set_incoming_angle(&mut self, angle: f64) {
        self.incoming_angle = Some(angle);
    }

    #[wasm_bindgen]
    pub fn clear_incoming_angle(&mut self) {
        self.incoming_angle = None;
    }

    #[wasm_bindgen]
    pub fn outgoing_angle(&self) -> Option<f64> {
        self.outgoing_angle
    }

    #[wasm_bindgen]
    pub fn set_outgoing_angle(&mut self, angle: f64) {
        self.outgoing_angle = Some(angle);
    }

    #[wasm_bindgen]
    pub fn clear_outgoing_angle(&mut self) {
        self.outgoing_angle = None;
    }
}

/// Web-friendly representation of skeleton information for curve fitting
///
/// The skeleton path is the original control polygon that should be preserved
/// during stroke refitting. This struct passes the skeleton and its point types
/// to the skeleton-aware refitting algorithm.
#[wasm_bindgen]
pub struct WebSkeletonInfo {
    /// Path data as SVG string (e.g., "M 10 10 L 20 20 L 30 10")
    path_data: String,

    /// Point types at each on-curve point (Smooth or Corner)
    point_types: Vec<WebPointType>,
}

#[wasm_bindgen]
impl WebSkeletonInfo {
    /// Create a new skeleton info from SVG path data
    ///
    /// # Arguments
    /// * `path_data` - SVG path string (e.g., "M 10 10 L 20 20 C 30 30 40 40 50 50")
    /// * `point_types` - Array of point types (0 for Smooth, 1 for Corner)
    #[wasm_bindgen(constructor)]
    pub fn new(path_data: String, point_types: Vec<u32>) -> Result<WebSkeletonInfo, JsValue> {
        let web_types: Vec<WebPointType> = point_types
            .iter()
            .map(|&t| {
                if t == 0 {
                    WebPointType::Smooth
                } else {
                    WebPointType::Corner
                }
            })
            .collect();

        Ok(WebSkeletonInfo {
            path_data,
            point_types: web_types,
        })
    }

    #[wasm_bindgen(getter)]
    pub fn path_data(&self) -> String {
        self.path_data.clone()
    }

    #[wasm_bindgen(getter)]
    pub fn point_types(&self) -> Vec<u32> {
        self.point_types
            .iter()
            .map(|&t| {
                if matches!(t, WebPointType::Smooth) {
                    0
                } else {
                    1
                }
            })
            .collect()
    }
}

#[wasm_bindgen]
pub struct CurveFitterOptions {
    cyclic: bool,
}

#[wasm_bindgen]
impl CurveFitterOptions {
    #[wasm_bindgen(constructor)]
    pub fn new() -> CurveFitterOptions {
        CurveFitterOptions { cyclic: false }
    }

    #[wasm_bindgen]
    pub fn set_cyclic(&mut self, cyclic: bool) {
        self.cyclic = cyclic;
    }
}

#[wasm_bindgen]
pub struct StrokeOptions {
    widths: Vec<f64>,
    cap: Cap,
    join: Join,
    interpolatable: bool,
    tolerance: f64,
}

#[wasm_bindgen]
impl StrokeOptions {
    #[wasm_bindgen(constructor)]
    pub fn new() -> StrokeOptions {
        StrokeOptions {
            widths: vec![10.0],
            cap: Cap::Round,
            join: Join::Round,
            interpolatable: false,
            tolerance: 0.1,
        }
    }

    #[wasm_bindgen]
    pub fn set_width(&mut self, width: f64) {
        self.widths = vec![width];
    }

    #[wasm_bindgen]
    pub fn set_widths(&mut self, widths: Vec<f64>) {
        self.widths = widths;
    }

    #[wasm_bindgen]
    pub fn set_tolerance(&mut self, tolerance: f64) {
        self.tolerance = tolerance;
    }

    #[wasm_bindgen]
    pub fn set_cap_butt(&mut self) {
        self.cap = Cap::Butt;
    }

    #[wasm_bindgen]
    pub fn set_cap_round(&mut self) {
        self.cap = Cap::Round;
    }

    #[wasm_bindgen]
    pub fn set_cap_square(&mut self) {
        self.cap = Cap::Square;
    }

    #[wasm_bindgen]
    pub fn set_join_bevel(&mut self) {
        self.join = Join::Bevel;
    }

    #[wasm_bindgen]
    pub fn set_join_miter(&mut self) {
        self.join = Join::Miter;
    }

    #[wasm_bindgen]
    pub fn set_join_round(&mut self) {
        self.join = Join::Round;
    }

    #[wasm_bindgen]
    pub fn set(&mut self, interpolatable: bool) {
        self.interpolatable = interpolatable;
    }
}

#[wasm_bindgen]
pub struct CurveSegment {
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
impl CurveSegment {
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
}

fn convert_web_point_type(web_type: WebPointType) -> PointType {
    match web_type {
        WebPointType::Smooth => PointType::Smooth,
        WebPointType::Corner => PointType::Corner,
    }
}

/// Parse SVG path string into a BezPath
fn parse_svg_path(path_data: &str) -> Result<BezPath, String> {
    BezPath::from_svg(path_data).map_err(|_| format!("Failed to parse SVG path: {}", path_data))
}

/// Create InputPoints from skeleton path for registration
fn skeleton_path_to_input_points(
    bez_path: &BezPath,
    point_types: &[WebPointType],
) -> Result<Vec<InputPoint>, String> {
    let mut points = Vec::new();
    let mut type_idx = 0;

    for element in bez_path.elements() {
        match element {
            kurbo::PathEl::MoveTo(pt) => {
                if type_idx < point_types.len() {
                    points.push(InputPoint {
                        x: pt.x,
                        y: pt.y,
                        point_type: convert_web_point_type(point_types[type_idx]),
                        incoming_angle: None,
                        outgoing_angle: None,
                    });
                    type_idx += 1;
                }
            }
            kurbo::PathEl::LineTo(pt)
            | kurbo::PathEl::QuadTo(_, pt)
            | kurbo::PathEl::CurveTo(_, _, pt) => {
                if type_idx < point_types.len() {
                    points.push(InputPoint {
                        x: pt.x,
                        y: pt.y,
                        point_type: convert_web_point_type(point_types[type_idx]),
                        incoming_angle: None,
                        outgoing_angle: None,
                    });
                    type_idx += 1;
                }
            }
            kurbo::PathEl::ClosePath => {
                // ClosePath doesn't add a point
            }
        }
    }

    if points.is_empty() {
        return Err("No points extracted from skeleton path".to_string());
    }

    Ok(points)
}

fn bez_path_to_segments(bez_path: &BezPath) -> Vec<CurveSegment> {
    let mut segments = Vec::new();
    let mut current_pos = Point::new(0.0, 0.0);

    for element in bez_path.elements() {
        match element {
            kurbo::PathEl::MoveTo(pt) => {
                current_pos = *pt;
            }
            kurbo::PathEl::LineTo(pt) => {
                segments.push(CurveSegment {
                    start_x: current_pos.x,
                    start_y: current_pos.y,
                    cp1_x: current_pos.x + (pt.x - current_pos.x) / 3.0,
                    cp1_y: current_pos.y + (pt.y - current_pos.y) / 3.0,
                    cp2_x: current_pos.x + 2.0 * (pt.x - current_pos.x) / 3.0,
                    cp2_y: current_pos.y + 2.0 * (pt.y - current_pos.y) / 3.0,
                    end_x: pt.x,
                    end_y: pt.y,
                });
                current_pos = *pt;
            }
            kurbo::PathEl::CurveTo(cp1, cp2, end) => {
                segments.push(CurveSegment {
                    start_x: current_pos.x,
                    start_y: current_pos.y,
                    cp1_x: cp1.x,
                    cp1_y: cp1.y,
                    cp2_x: cp2.x,
                    cp2_y: cp2.y,
                    end_x: end.x,
                    end_y: end.y,
                });
                current_pos = *end;
            }
            kurbo::PathEl::QuadTo(cp, end) => {
                let cp1 = current_pos + (2.0 / 3.0) * (*cp - current_pos);
                let cp2 = *end + (2.0 / 3.0) * (*cp - *end);

                segments.push(CurveSegment {
                    start_x: current_pos.x,
                    start_y: current_pos.y,
                    cp1_x: cp1.x,
                    cp1_y: cp1.y,
                    cp2_x: cp2.x,
                    cp2_y: cp2.y,
                    end_x: end.x,
                    end_y: end.y,
                });
                current_pos = *end;
            }
            kurbo::PathEl::ClosePath => {
                continue;
            }
        }
    }

    segments
}

#[wasm_bindgen]
pub fn fit_curve(points: Vec<WebPoint>, options: &CurveFitterOptions) -> Vec<CurveSegment> {
    console_error_panic_hook::set_once();

    if points.len() < 2 {
        return Vec::new();
    }

    let input_points: Vec<InputPoint> = points
        .iter()
        .map(|p| InputPoint {
            x: p.x,
            y: p.y,
            point_type: convert_web_point_type(p.point_type),
            incoming_angle: p.incoming_angle,
            outgoing_angle: p.outgoing_angle,
        })
        .collect();

    match curve_fitter::fit_curve(input_points, options.cyclic) {
        Ok(bez_path) => bez_path_to_segments(&bez_path),
        Err(_) => Vec::new(),
    }
}

#[wasm_bindgen]
pub fn fit_curve_with_stroke(
    points: Vec<WebPoint>,
    curve_options: &CurveFitterOptions,
    stroke_options: &StrokeOptions,
) -> Vec<CurveSegment> {
    console_error_panic_hook::set_once();

    if points.len() < 2 {
        return Vec::new();
    }

    let input_points: Vec<InputPoint> = points
        .iter()
        .map(|p| InputPoint {
            x: p.x,
            y: p.y,
            point_type: convert_web_point_type(p.point_type),
            incoming_angle: p.incoming_angle,
            outgoing_angle: p.outgoing_angle,
        })
        .collect();

    match curve_fitter::fit_curve(input_points, curve_options.cyclic) {
        Ok(bez_path) => {
            let widths = &stroke_options.widths;

            if widths.is_empty() {
                console_log!("Error: No widths provided for stroke");
                return Vec::new();
            }

            let all_same =
                widths.len() == 1 || widths.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-10);

            let stroked = if all_same {
                let style = Stroke::new(widths[0])
                    .with_caps(stroke_options.cap)
                    .with_join(stroke_options.join);
                stroke(
                    bez_path,
                    &style,
                    &StrokeOpts::default(),
                    stroke_options.tolerance,
                )
            } else {
                let style = VariableStroke::default()
                    .with_caps(stroke_options.cap)
                    .with_join(stroke_options.join);

                let stroker = VariableStroker::new(stroke_options.tolerance);
                stroker.stroke(&bez_path, widths, &style)
            };

            // Refit the stroke outline to get cleaner curves
            let final_path = match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                Ok(refitted) => {
                    console_log!(
                        "Stroke outline refitted: {} → {} elements",
                        stroked.elements().len(),
                        refitted.elements().len()
                    );
                    refitted
                }
                Err(e) => {
                    console_log!("Warning: Failed to refit stroke ({}), using original", e);
                    stroked
                }
            };

            bez_path_to_segments(&final_path)
        }
        Err(e) => {
            console_log!("Error fitting curve with stroke: {}", e);
            Vec::new()
        }
    }
}

#[wasm_bindgen]
pub fn curve_to_svg_path(points: Vec<WebPoint>, options: &CurveFitterOptions) -> String {
    console_error_panic_hook::set_once();

    if points.len() < 2 {
        return String::new();
    }

    let input_points: Vec<InputPoint> = points
        .iter()
        .map(|p| InputPoint {
            x: p.x,
            y: p.y,
            point_type: convert_web_point_type(p.point_type),
            incoming_angle: p.incoming_angle,
            outgoing_angle: p.outgoing_angle,
        })
        .collect();

    match curve_fitter::fit_curve(input_points, options.cyclic) {
        Ok(bez_path) => bez_path.to_svg(),
        Err(e) => {
            console_log!("Error fitting curve with stroke: {}", e);
            String::new()
        }
    }
}

#[wasm_bindgen]
pub fn curve_to_svg_path_with_stroke(
    points: Vec<WebPoint>,
    curve_options: &CurveFitterOptions,
    stroke_options: &StrokeOptions,
    use_stroke: bool,
) -> String {
    console_error_panic_hook::set_once();

    if points.len() < 2 {
        return String::new();
    }

    let input_points: Vec<InputPoint> = points
        .iter()
        .map(|p| InputPoint {
            x: p.x,
            y: p.y,
            point_type: convert_web_point_type(p.point_type),
            incoming_angle: p.incoming_angle,
            outgoing_angle: p.outgoing_angle,
        })
        .collect();

    match curve_fitter::fit_curve(input_points, curve_options.cyclic) {
        Ok(bez_path) => {
            if use_stroke {
                let widths = &stroke_options.widths;

                if widths.is_empty() {
                    console_log!("Error: No widths provided for stroke");
                    return String::new();
                }

                let all_same =
                    widths.len() == 1 || widths.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-10);

                let stroked = if all_same {
                    let style = Stroke::new(widths[0])
                        .with_caps(stroke_options.cap)
                        .with_join(stroke_options.join);
                    stroke(
                        bez_path,
                        &style,
                        &StrokeOpts::default(),
                        stroke_options.tolerance,
                    )
                } else {
                    let style = VariableStroke::default()
                        .with_caps(stroke_options.cap)
                        .with_join(stroke_options.join);

                    let stroker = VariableStroker::new(stroke_options.tolerance);
                    stroker.stroke(&bez_path, widths, &style)
                };

                // Refit the stroke outline to get cleaner curves
                match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                    Ok(refitted) => {
                        console_log!(
                            "Stroke outline refitted: {} → {} elements",
                            stroked.elements().len(),
                            refitted.elements().len()
                        );
                        refitted.to_svg()
                    }
                    Err(e) => {
                        console_log!("Warning: Failed to refit stroke ({}), using original", e);
                        stroked.to_svg()
                    }
                }
            } else {
                bez_path.to_svg()
            }
        }
        Err(e) => {
            console_log!("Error fitting curve with stroke: {}", e);
            String::new()
        }
    }
}

/// Fit curve with stroke and skeleton-guided refitting for improved accuracy
///
/// This function combines the standard stroke refitting with skeleton information
/// to produce curves that closely match the original design intent. The skeleton
/// acts as a guide for correcting any false corners or misclassifications that
/// occur during the stroking process.
///
/// # Arguments
/// * `points` - Input points for curve fitting
/// * `curve_options` - Curve fitting configuration
/// * `stroke_options` - Stroke parameters (width, caps, joins)
/// * `skeleton_info` - Skeleton path with point type information (optional)
///
/// # Returns
/// Vector of curve segments representing the refitted stroke
#[wasm_bindgen]
pub fn fit_curve_with_stroke_and_skeleton(
    points: Vec<WebPoint>,
    curve_options: &CurveFitterOptions,
    stroke_options: &StrokeOptions,
    skeleton_info: Option<WebSkeletonInfo>,
) -> Vec<CurveSegment> {
    console_error_panic_hook::set_once();

    if points.len() < 2 {
        return Vec::new();
    }

    let input_points: Vec<InputPoint> = points
        .iter()
        .map(|p| InputPoint {
            x: p.x,
            y: p.y,
            point_type: convert_web_point_type(p.point_type),
            incoming_angle: p.incoming_angle,
            outgoing_angle: p.outgoing_angle,
        })
        .collect();

    match curve_fitter::fit_curve(input_points, curve_options.cyclic) {
        Ok(bez_path) => {
            let widths = &stroke_options.widths;

            if widths.is_empty() {
                console_log!("Error: No widths provided for stroke");
                return Vec::new();
            }

            let all_same =
                widths.len() == 1 || widths.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-10);

            let stroked = if all_same {
                let style = Stroke::new(widths[0])
                    .with_caps(stroke_options.cap)
                    .with_join(stroke_options.join);
                stroke(
                    bez_path,
                    &style,
                    &StrokeOpts::default(),
                    stroke_options.tolerance,
                )
            } else {
                let style = VariableStroke::default()
                    .with_caps(stroke_options.cap)
                    .with_join(stroke_options.join);

                let stroker = VariableStroker::new(stroke_options.tolerance);
                stroker.stroke(&bez_path, widths, &style)
            };

            // Apply skeleton-aware refitting if skeleton is provided
            let final_path = if let Some(skeleton) = skeleton_info {
                match parse_svg_path(&skeleton.path_data) {
                    Ok(skeleton_path) => {
                        match skeleton_path_to_input_points(&skeleton_path, &skeleton.point_types) {
                            Ok(skeleton_points) => {
                                match register_skeleton_for_preservation(
                                    &skeleton_path,
                                    &skeleton_points,
                                    widths,
                                    curve_options.cyclic,
                                ) {
                                    Ok(skeleton_info_obj) => {
                                        match refit_stroke_with_skeleton_correction(
                                            &stroked,
                                            &skeleton_info_obj,
                                            &StrokeRefitterConfig::new(),
                                        ) {
                                            Ok(refitted) => {
                                                console_log!(
                                                    "Stroke refitted with skeleton correction: {} → {} elements",
                                                    stroked.elements().len(),
                                                    refitted.elements().len()
                                                );
                                                refitted
                                            }
                                            Err(e) => {
                                                console_log!(
                                                    "Warning: Skeleton correction failed ({}), using outline refitting",
                                                    e
                                                );
                                                match refit_stroke(
                                                    &stroked,
                                                    &StrokeRefitterConfig::new(),
                                                ) {
                                                    Ok(refitted) => refitted,
                                                    Err(e2) => {
                                                        console_log!(
                                                            "Warning: Outline refitting also failed ({}), using original",
                                                            e2
                                                        );
                                                        stroked
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        console_log!(
                                            "Warning: Failed to register skeleton ({}), using outline refitting",
                                            e
                                        );
                                        match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                                            Ok(refitted) => refitted,
                                            Err(e2) => {
                                                console_log!(
                                                    "Warning: Outline refitting failed ({}), using original",
                                                    e2
                                                );
                                                stroked
                                            }
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                console_log!(
                                    "Warning: Failed to extract skeleton points ({}), using outline refitting",
                                    e
                                );
                                match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                                    Ok(refitted) => refitted,
                                    Err(e2) => {
                                        console_log!(
                                            "Warning: Outline refitting failed ({}), using original",
                                            e2
                                        );
                                        stroked
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        console_log!(
                            "Warning: Failed to parse skeleton path ({}), using outline refitting",
                            e
                        );
                        match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                            Ok(refitted) => refitted,
                            Err(e2) => {
                                console_log!(
                                    "Warning: Outline refitting failed ({}), using original",
                                    e2
                                );
                                stroked
                            }
                        }
                    }
                }
            } else {
                // No skeleton provided, use outline-based refitting
                match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                    Ok(refitted) => {
                        console_log!(
                            "Stroke outline refitted: {} → {} elements",
                            stroked.elements().len(),
                            refitted.elements().len()
                        );
                        refitted
                    }
                    Err(e) => {
                        console_log!("Warning: Failed to refit stroke ({}), using original", e);
                        stroked
                    }
                }
            };

            bez_path_to_segments(&final_path)
        }
        Err(e) => {
            console_log!("Error fitting curve with stroke: {}", e);
            Vec::new()
        }
    }
}

/// Convert curve to SVG path with stroke and skeleton-guided refitting
///
/// This is the SVG path variant of fit_curve_with_stroke_and_skeleton.
/// Returns an SVG path string instead of curve segments.
#[wasm_bindgen]
pub fn curve_to_svg_path_with_stroke_and_skeleton(
    points: Vec<WebPoint>,
    curve_options: &CurveFitterOptions,
    stroke_options: &StrokeOptions,
    skeleton_info: Option<WebSkeletonInfo>,
) -> String {
    console_error_panic_hook::set_once();

    if points.len() < 2 {
        return String::new();
    }

    let input_points: Vec<InputPoint> = points
        .iter()
        .map(|p| InputPoint {
            x: p.x,
            y: p.y,
            point_type: convert_web_point_type(p.point_type),
            incoming_angle: p.incoming_angle,
            outgoing_angle: p.outgoing_angle,
        })
        .collect();

    match curve_fitter::fit_curve(input_points, curve_options.cyclic) {
        Ok(bez_path) => {
            let widths = &stroke_options.widths;

            if widths.is_empty() {
                console_log!("Error: No widths provided for stroke");
                return String::new();
            }

            let all_same =
                widths.len() == 1 || widths.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-10);

            let stroked = if all_same {
                let style = Stroke::new(widths[0])
                    .with_caps(stroke_options.cap)
                    .with_join(stroke_options.join);
                stroke(
                    bez_path,
                    &style,
                    &StrokeOpts::default(),
                    stroke_options.tolerance,
                )
            } else {
                let style = VariableStroke::default()
                    .with_caps(stroke_options.cap)
                    .with_join(stroke_options.join);

                let stroker = VariableStroker::new(stroke_options.tolerance);
                stroker.stroke(&bez_path, widths, &style)
            };

            // Apply skeleton-aware refitting if skeleton is provided
            let final_path = if let Some(skeleton) = skeleton_info {
                match parse_svg_path(&skeleton.path_data) {
                    Ok(skeleton_path) => {
                        match skeleton_path_to_input_points(&skeleton_path, &skeleton.point_types) {
                            Ok(skeleton_points) => {
                                match register_skeleton_for_preservation(
                                    &skeleton_path,
                                    &skeleton_points,
                                    widths,
                                    curve_options.cyclic,
                                ) {
                                    Ok(skeleton_info_obj) => {
                                        match refit_stroke_with_skeleton_correction(
                                            &stroked,
                                            &skeleton_info_obj,
                                            &StrokeRefitterConfig::new(),
                                        ) {
                                            Ok(refitted) => {
                                                console_log!(
                                                    "Stroke refitted with skeleton correction: {} → {} elements",
                                                    stroked.elements().len(),
                                                    refitted.elements().len()
                                                );
                                                refitted
                                            }
                                            Err(e) => {
                                                console_log!(
                                                    "Warning: Skeleton correction failed ({}), using outline refitting",
                                                    e
                                                );
                                                match refit_stroke(
                                                    &stroked,
                                                    &StrokeRefitterConfig::new(),
                                                ) {
                                                    Ok(refitted) => refitted,
                                                    Err(e2) => {
                                                        console_log!(
                                                            "Warning: Outline refitting also failed ({}), using original",
                                                            e2
                                                        );
                                                        stroked
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        console_log!(
                                            "Warning: Failed to register skeleton ({}), using outline refitting",
                                            e
                                        );
                                        match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                                            Ok(refitted) => refitted,
                                            Err(e2) => {
                                                console_log!(
                                                    "Warning: Outline refitting failed ({}), using original",
                                                    e2
                                                );
                                                stroked
                                            }
                                        }
                                    }
                                }
                            }
                            Err(e) => {
                                console_log!(
                                    "Warning: Failed to extract skeleton points ({}), using outline refitting",
                                    e
                                );
                                match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                                    Ok(refitted) => refitted,
                                    Err(e2) => {
                                        console_log!(
                                            "Warning: Outline refitting failed ({}), using original",
                                            e2
                                        );
                                        stroked
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        console_log!(
                            "Warning: Failed to parse skeleton path ({}), using outline refitting",
                            e
                        );
                        match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                            Ok(refitted) => refitted,
                            Err(e2) => {
                                console_log!(
                                    "Warning: Outline refitting failed ({}), using original",
                                    e2
                                );
                                stroked
                            }
                        }
                    }
                }
            } else {
                // No skeleton provided, use outline-based refitting
                match refit_stroke(&stroked, &StrokeRefitterConfig::new()) {
                    Ok(refitted) => {
                        console_log!(
                            "Stroke outline refitted: {} → {} elements",
                            stroked.elements().len(),
                            refitted.elements().len()
                        );
                        refitted
                    }
                    Err(e) => {
                        console_log!("Warning: Failed to refit stroke ({}), using original", e);
                        stroked
                    }
                }
            };

            final_path.to_svg()
        }
        Err(e) => {
            console_log!("Error fitting curve with stroke: {}", e);
            String::new()
        }
    }
}

#[wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
}
