use std::collections::HashMap;

use hobby::hobby;
use hobby::point::Point;
fn main() {
    let points = vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 0.0),
        Point::new(200.0, 0.0),
    ];
    let mut exit_angles = HashMap::new();
    exit_angles.insert(0, 45.0);

    let mut entry_angles = HashMap::new();
    entry_angles.insert(1, -90.0);
    entry_angles.insert(2, 90.0);
    let segments = hobby(&points, None, true, Some(&entry_angles), Some(&exit_angles));
    for (i, seg) in segments.iter().enumerate() {
        println!(
            "Segment {}: Start=({:.2}, {:.2}), C1=({:.2}, {:.2}), C2=({:.2}, {:.2}), End=({:.2}, {:.2})",
            i, seg.0.x, seg.0.y, seg.1.x, seg.1.y, seg.2.x, seg.2.y, seg.3.x, seg.3.y
        );
    }
}
