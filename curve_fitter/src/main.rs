use curve_fitter::{CurveFitter, InputPoint, PointType};
use std::fs;

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
            point_type: PointType::Corner,
        },
        InputPoint {
            x: 350.0,
            y: 100.0,
            point_type: PointType::Smooth,
        },
    ];

    let fitter = CurveFitter::new();

    match fitter.fit_curve(input_points, true) {
        Ok(bez_path) => {
            println!("Successfully fitted curve!");
            println!(
                "Bezier path created with {} elements",
                bez_path.elements().len()
            );

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
            match fs::write("curve-fit.svg", svg_content) {
                Ok(()) => println!("SVG file written to curve-fit.svg"),
                Err(e) => println!("Error writing SVG file: {}", e),
            }
        }
        Err(e) => {
            println!("Error fitting curve: {}", e);
        }
    }
}
