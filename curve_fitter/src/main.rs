use curve_fitter::{
    CurveFitter, InputPoint, PointType, var_stroke::VariableStroke, var_stroker::VariableStroker,
};
use kurbo::{
    BezPath, ParamCurveFit, PathEl, fit_to_bezpath_opt,
    simplify::{SimplifyOptLevel, SimplifyOptions, simplify_bezpath},
};
use std::fs;

fn write_path_to_svg(bez_path: &BezPath, output_file_name: &str) {
    // Create a complete SVG document
    let path_data = bez_path.to_svg();
    let svg_content = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<svg width="400" height="200" viewBox="0 0 400 200" xmlns="http://www.w3.org/2000/svg">
  <path d="{}" stroke="orange" stroke-width="2" fill="none"/>
</svg>"#,
        path_data
    );

    // Write to file
    match fs::write(output_file_name, svg_content) {
        Ok(()) => println!("SVG file written to {}", output_file_name),
        Err(e) => println!("Error writing SVG file: {}", e),
    }
}

fn count_points(path: &BezPath) -> usize {
    path.iter()
        .filter(|el| {
            matches!(
                el,
                PathEl::MoveTo(_)
                    | PathEl::LineTo(_)
                    | PathEl::QuadTo(_, _)
                    | PathEl::CurveTo(_, _, _)
            )
        })
        .count()
}

fn main() {
    println!("Curve Fitting Demo");

    // Create some sample input points - a simple curved path
    let input_points = vec![
        InputPoint {
            x: 50.0,
            y: 100.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 150.0,
            y: 50.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 250.0,
            y: 150.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 350.0,
            y: 100.0,
            point_type: PointType::Smooth,
        },
    ];

    let fitter = CurveFitter::new();

    let style = VariableStroke::default();
    match fitter.fit_curve(input_points, false) {
        Ok(bez_path) => {
            println!("Successfully fitted curve!");
            println!(
                "Bezier path created with {} elements",
                bez_path.elements().len()
            );
            write_path_to_svg(&bez_path, "curve-fit.svg");
            // All same width - should behave like constant stroke
            let widths = vec![5.0, 5.0, 5.0, 5.0];
            let stroke = VariableStroker::new(0.1);
            let result_path = stroke.stroke(&bez_path, &widths, &style);
            write_path_to_svg(&result_path, "curve-fit-same-stroke.svg");
            // Varying same width - should behave like constant stroke
            let widths = vec![10.0, 15.0, 20.0, 25.0];
            let stroke = VariableStroker::new(0.1);
            let result_path = stroke.stroke(&bez_path, &widths, &style);
            write_path_to_svg(&result_path, "curve-fit-var-stroke.svg");
            println!("Output has {} curve segments", count_points(&result_path));
        }
        Err(e) => {
            println!("Error fitting curve: {}", e);
        }
    }
    // Approximate letter 'O' with 4 cubic curves
    let mut o_path = BezPath::new();
    o_path.move_to((50.0, 0.0));
    // Top right
    o_path.curve_to((77.6, 0.0), (100.0, 22.4), (100.0, 50.0));
    // Bottom right
    o_path.curve_to((100.0, 77.6), (77.6, 100.0), (50.0, 100.0));
    // Bottom left
    o_path.curve_to((22.4, 100.0), (0.0, 77.6), (0.0, 50.0));
    // Top left
    o_path.curve_to((0.0, 22.4), (22.4, 0.0), (50.0, 0.0));
    o_path.close_path();

    // Thick on left/right (vertical stems), thin on top/bottom (horizontal)
    let widths = vec![
        10.0, // right
        5.0,  // bottom
        10.0, // left
        5.0,  // top
    ];

    let stroker = VariableStroker::new(0.1);

    let result_path = stroker.stroke(&o_path, &widths, &style);
    let options = SimplifyOptions::default().opt_level(SimplifyOptLevel::Optimize);
    let simplified_path = simplify_bezpath(&result_path, 0.1, &options);
    write_path_to_svg(&result_path, "curve-fit-o-stroke.svg");
    println!("Output has {} curve segments", count_points(&result_path));
    write_path_to_svg(&simplified_path, "curve-fit-o-stroke-simplified.svg");
    println!(
        "Simplified Output has {} curve segments",
        count_points(&simplified_path)
    );

    let simpl = kurbo::simplify::SimplifyBezPath::new(result_path);
    let fitted_path = kurbo::fit_to_bezpath_opt(&simpl, 0.1);

    write_path_to_svg(&fitted_path, "curve-fit-o-stroke-fitted.svg");

    println!("Output has {} curve segments", count_points(&fitted_path));
}
