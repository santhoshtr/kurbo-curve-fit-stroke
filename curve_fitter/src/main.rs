use curve_fitter::{
    InputPoint, PointType, fit_curve, var_stroke::VariableStroke, var_stroker::VariableStroker,
};
use kurbo::{BezPath, PathEl};
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

fn refit_stroke(stroke_path: &BezPath) -> Result<BezPath, String> {
    let mut refitted_input_points = Vec::new();
    for el in stroke_path.iter() {
        match el {
            PathEl::MoveTo(p) => {
                refitted_input_points.push(InputPoint {
                    x: p.x,
                    y: p.y,
                    point_type: PointType::Smooth,
                });
            }
            PathEl::LineTo(p) => {
                refitted_input_points.push(InputPoint {
                    x: p.x,
                    y: p.y,
                    point_type: PointType::Smooth,
                });
            }
            PathEl::QuadTo(_, p2) => {
                refitted_input_points.push(InputPoint {
                    x: p2.x,
                    y: p2.y,
                    point_type: PointType::Smooth,
                });
            }
            PathEl::CurveTo(_, _, p3) => {
                refitted_input_points.push(InputPoint {
                    x: p3.x,
                    y: p3.y,
                    point_type: PointType::Smooth,
                });
            }
            PathEl::ClosePath => {
                dbg!("close");
            }
        }
    }

    let mut deduped_points = Vec::new();
    let epsilon = 1e-6;
    for pt in refitted_input_points {
        let is_duplicate = deduped_points.last().map_or_else(
            || false,
            |last: &InputPoint| (pt.x - last.x).abs() < epsilon && (pt.y - last.y).abs() < epsilon,
        );
        if !is_duplicate {
            deduped_points.push(pt);
        }
    }
    let refitted_input_points = deduped_points;

    println!(
        "Refitting {} on-curve points from result_path",
        refitted_input_points.len()
    );
    for (i, pt) in refitted_input_points.iter().enumerate() {
        println!(
            "  [{}] InputPoint {{ x: {}, y: {}, point_type: {:?} }}",
            i, pt.x, pt.y, pt.point_type
        );
    }

    fit_curve(refitted_input_points, false)
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

    let style = VariableStroke::default();
    match fit_curve(input_points, false) {
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
            match refit_stroke(&result_path) {
                Ok(refitted_bez_path) => {
                    println!("Successfully refitted curve from result_path!");
                    println!(
                        "Refitted bezier path created with {} elements",
                        refitted_bez_path.elements().len()
                    );
                    write_path_to_svg(&refitted_bez_path, "curve-fit-stroke-refitted.svg");
                }
                Err(e) => {
                    println!("Error refitting curve: {}", e);
                }
            }
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
    write_path_to_svg(&result_path, "curve-fit-o-stroke.svg");
    println!("Output has {} curve segments", count_points(&result_path));

    match refit_stroke(&result_path) {
        Ok(refitted_bez_path) => {
            println!("Successfully refitted curve from result_path!");
            println!(
                "Refitted bezier path created with {} elements",
                refitted_bez_path.elements().len()
            );
            write_path_to_svg(&refitted_bez_path, "curve-fit-o-stroke-refitted.svg");
        }
        Err(e) => {
            println!("Error refitting curve: {}", e);
        }
    }

    // Just a line
    let mut straight_line = BezPath::new();
    straight_line.move_to((0., 0.));
    straight_line.line_to((100., 100.));
    straight_line.close_path();
    let widths = vec![2.0, 10.0];
    let stroker = VariableStroker::new(0.1);
    let result_path = stroker.stroke(&straight_line, &widths, &style);
    write_path_to_svg(&result_path, "line-outline.svg");
}
