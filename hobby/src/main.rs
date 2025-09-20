use std::collections::HashMap;

use hobby::hobby;
use hobby::point::Point;
fn main() {
    println!("--- Example 1: (1,0) -> (0,1) with angle constraints ---");
    let points1 = vec![Point::new(1.0, 0.0), Point::new(0.0, 1.0)];
    let mut exit_angles1 = HashMap::new();
    exit_angles1.insert(0, 90.0); // Leaves point 0 at 45 degrees
    let mut entry_angles1 = HashMap::new();
    entry_angles1.insert(1, 180.0); // Arrives at point 1 at 270 degrees

    let segments1 = hobby(
        &points1,
        None,
        false,
        Some(&entry_angles1),
        Some(&exit_angles1),
    );
    for (i, seg) in segments1.iter().enumerate() {
        println!(
            "Segment {}: Start=({:.2}, {:.2}), C1=({:.2}, {:.2}), C2=({:.2}, {:.2}), End=({:.2}, {:.2})",
            i, seg.0.x, seg.0.y, seg.1.x, seg.1.y, seg.2.x, seg.2.y, seg.3.x, seg.3.y
        );
    }

    println!("\n--- Example 2: Three points with one angle constraint ---");
    let points2 = vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 100.0),
        Point::new(200.0, 0.0),
    ];
    let mut exit_angles2 = HashMap::new();
    exit_angles2.insert(1, -45.0); // Leaves the middle point at a downward 45-degree angle

    let segments2 = hobby(&points2, None, false, None, Some(&exit_angles2));
    for (i, seg) in segments2.iter().enumerate() {
        println!(
            "Segment {}: Start=({:.2}, {:.2}), C1=({:.2}, {:.2}), C2=({:.2}, {:.2}), End=({:.2}, {:.2})",
            i, seg.0.x, seg.0.y, seg.1.x, seg.1.y, seg.2.x, seg.2.y, seg.3.x, seg.3.y
        );
    }
}
