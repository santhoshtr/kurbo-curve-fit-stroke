use curve_fitter::{CurveFitter, InputPoint, PointType};

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

    match fitter.fit_curve(input_points, false) {
        Ok(bez_path) => {
            println!("Successfully fitted curve!");
            println!(
                "Bezier path created with {} elements",
                bez_path.elements().len()
            );

            // You can now use the BezPath for rendering or further processing
            // For example, convert to SVG path data:
            println!("SVG path: {}", bez_path.to_svg());
        }
        Err(e) => {
            println!("Error fitting curve: {}", e);
        }
    }

    // Test with a corner point
    let input_points_with_corner = vec![
        InputPoint {
            x: 100.0,
            y: 100.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 200.0,
            y: 100.0,
            point_type: PointType::Corner,
        },
        InputPoint {
            x: 200.0,
            y: 200.0,
            point_type: PointType::Smooth,
        },
    ];

    match fitter.fit_curve(input_points_with_corner, false) {
        Ok(bez_path) => {
            println!("\nSuccessfully fitted curve with corner!");
            println!("SVG path: {}", bez_path.to_svg());
        }
        Err(e) => {
            println!("Error fitting curve with corner: {}", e);
        }
    }

    // Test closed curve
    let closed_points = vec![
        InputPoint {
            x: 100.0,
            y: 100.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 200.0,
            y: 100.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 200.0,
            y: 200.0,
            point_type: PointType::Smooth,
        },
        InputPoint {
            x: 100.0,
            y: 200.0,
            point_type: PointType::Smooth,
        },
    ];

    match fitter.fit_curve(closed_points, true) {
        Ok(bez_path) => {
            println!("\nSuccessfully fitted closed curve!");
            println!("SVG path: {}", bez_path.to_svg());
        }
        Err(e) => {
            println!("Error fitting closed curve: {}", e);
        }
    }
}
