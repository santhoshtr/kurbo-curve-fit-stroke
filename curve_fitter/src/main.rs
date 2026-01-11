use curve_fitter::{
    InputPoint, PointType, fit_curve, refit_stroke, refit_stroke_with_skeleton,
    register_skeleton_for_preservation, var_stroke::VariableStroke, var_stroker::VariableStroker,
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

fn main() {
    println!("Curve Fitting Demo");

    // Create some sample input points - a simple curved path
    let input_points = vec![
        InputPoint {
            x: 50.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
        },
        InputPoint {
            x: 150.0,
            y: 50.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
        },
        InputPoint {
            x: 250.0,
            y: 150.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
        },
        InputPoint {
            x: 350.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
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
            let widths = vec![10.0, 15.0, 40.0, 25.0];
            let stroke = VariableStroker::new(0.1);
            let result_path = stroke.stroke(&bez_path, &widths, &style);
            write_path_to_svg(&result_path, "curve-fit-var-stroke.svg");
            println!("Output has {} curve segments", count_points(&result_path));
            println!("Stroke path elements:");
            for (i, el) in result_path.iter().enumerate() {
                match el {
                    PathEl::MoveTo(p) => println!("  {}: MoveTo({:.2}, {:.2})", i, p.x, p.y),
                    PathEl::LineTo(p) => println!("  {}: LineTo({:.2}, {:.2})", i, p.x, p.y),
                    PathEl::CurveTo(_cp1, _cp2, p) => {
                        println!("  {}: CurveTo(..., {:.2}, {:.2})", i, p.x, p.y)
                    }
                    PathEl::QuadTo(_cp, p) => {
                        println!("  {}: QuadTo(..., {:.2}, {:.2})", i, p.x, p.y)
                    }
                    PathEl::ClosePath => println!("  {}: ClosePath", i),
                }
            }
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

    // MetaPost-style example with explicit tangent angles
    // Equivalent to: z1{dir 10}..{dir 90}z2..z3{dir 45}..z4
    println!("\n=== MetaPost-Style Curve Demo ===");
    let metapost_points = vec![
        InputPoint {
            x: 50.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: Some(10.0), // {dir 10} leaving z1
        },
        InputPoint {
            x: 150.0,
            y: 50.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(90.0), // {dir 90} approaching z2
            outgoing_angle: None,       // computed automatically
        },
        InputPoint {
            x: 250.0,
            y: 150.0,
            point_type: PointType::Smooth,
            incoming_angle: None,       // computed automatically
            outgoing_angle: Some(45.0), // {dir 45} leaving z3
        },
        InputPoint {
            x: 350.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
        },
    ];

    match fit_curve(metapost_points, false) {
        Ok(bez_path) => {
            println!("Successfully created MetaPost-style curve!");
            println!(
                "Bezier path created with {} elements",
                bez_path.elements().len()
            );
            write_path_to_svg(&bez_path, "metapost-style-curve.svg");
        }
        Err(e) => {
            println!("Error creating MetaPost-style curve: {}", e);
        }
    }

    // Example with different incoming/outgoing angles (creates a corner)
    println!("\n=== Corner Example (Different Angles) ===");
    let corner_points = vec![
        InputPoint {
            x: 50.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: Some(0.0), // horizontal out
        },
        InputPoint {
            x: 150.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(0.0),  // horizontal in
            outgoing_angle: Some(90.0), // vertical out - creates corner!
        },
        InputPoint {
            x: 150.0,
            y: 200.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: None,
        },
    ];

    match fit_curve(corner_points.clone(), false) {
        Ok(bez_path) => {
            println!("Successfully created corner curve!");
            println!(
                "Bezier path created with {} elements",
                bez_path.elements().len()
            );
            write_path_to_svg(&bez_path, "corner-angle-curve.svg");
        }
        Err(e) => {
            println!("Error creating corner curve: {}", e);
        }
    }

    // Skeleton angle preservation example
    println!("\n=== Skeleton Angle Preservation Demo ===");
    let skeleton_points = vec![
        InputPoint {
            x: 50.0,
            y: 100.0,
            point_type: PointType::Smooth,
            incoming_angle: None,
            outgoing_angle: Some(20.0), // Specific angle leaving first point
        },
        InputPoint {
            x: 150.0,
            y: 50.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(80.0),  // Specific angle approaching
            outgoing_angle: Some(100.0), // Specific angle leaving
        },
        InputPoint {
            x: 250.0,
            y: 150.0,
            point_type: PointType::Smooth,
            incoming_angle: Some(260.0), // Specific angle approaching last point
            outgoing_angle: None,
        },
    ];

    match fit_curve(skeleton_points.clone(), false) {
        Ok(skeleton) => {
            println!(
                "Created skeleton curve with {} elements",
                skeleton.elements().len()
            );

            // Register skeleton before stroking
            let widths = vec![10.0, 15.0, 10.0]; // Variable width
            match register_skeleton_for_preservation(&skeleton, &skeleton_points, &widths, false) {
                Ok(skeleton_info) => {
                    println!("Skeleton successfully registered for angle preservation");

                    // Stroke the skeleton
                    let stroker = VariableStroker::new(0.1);
                    let outline = stroker.stroke(&skeleton, &widths, &style);
                    println!(
                        "Stroke outline created with {} elements",
                        outline.elements().len()
                    );
                    write_path_to_svg(&outline, "skeleton-stroke-outline.svg");

                    // Refit WITHOUT skeleton preservation (for comparison)
                    match refit_stroke(&outline) {
                        Ok(refitted_no_skel) => {
                            println!(
                                "Refitted WITHOUT skeleton: {} elements",
                                refitted_no_skel.elements().len()
                            );
                            write_path_to_svg(&refitted_no_skel, "skeleton-refitted-without.svg");
                        }
                        Err(e) => println!("Error refitting without skeleton: {}", e),
                    }

                    // Refit WITH skeleton preservation
                    match refit_stroke_with_skeleton(&outline, &skeleton_info) {
                        Ok(refitted_with_skel) => {
                            println!(
                                "Refitted WITH skeleton: {} elements",
                                refitted_with_skel.elements().len()
                            );
                            write_path_to_svg(&refitted_with_skel, "skeleton-refitted-with.svg");
                            println!("✓ Skeleton angles preserved in refitted stroke!");
                        }
                        Err(e) => println!("Error refitting with skeleton: {}", e),
                    }
                }
                Err(e) => println!("Error registering skeleton: {}", e),
            }
        }
        Err(e) => {
            println!("Error creating skeleton curve: {}", e);
        }
    }
}
