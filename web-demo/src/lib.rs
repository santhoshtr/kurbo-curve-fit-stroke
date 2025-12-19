use curve_fitter::{
    CurveFitter, InputPoint, PointType, var_stroke::VariableStroke, var_stroker::VariableStroker,
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
}

#[wasm_bindgen]
impl WebPoint {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64) -> WebPoint {
        WebPoint {
            x,
            y,
            point_type: WebPointType::Smooth,
        }
    }

    #[wasm_bindgen]
    pub fn new_with_type(x: f64, y: f64, point_type: WebPointType) -> WebPoint {
        WebPoint { x, y, point_type }
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
        })
        .collect();

    let fitter = CurveFitter::new();

    match fitter.fit_curve(input_points, options.cyclic) {
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
        })
        .collect();

    let fitter = CurveFitter::new();

    match fitter.fit_curve(input_points, curve_options.cyclic) {
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

            bez_path_to_segments(&stroked)
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
        })
        .collect();

    let fitter = CurveFitter::new();

    match fitter.fit_curve(input_points, options.cyclic) {
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
        })
        .collect();

    let fitter = CurveFitter::new();

    match fitter.fit_curve(input_points, curve_options.cyclic) {
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

                stroked.to_svg()
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

#[wasm_bindgen(start)]
pub fn main() {
    console_error_panic_hook::set_once();
}
