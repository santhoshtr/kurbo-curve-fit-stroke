use std::collections::HashMap;

use hobby::hobby;
use hobby::point::Point;
fn main() {
    let points = vec![
        Point::new(0.0, 0.0),
        Point::new(100.0, 100.0),
        Point::new(200.0, 0.0),
    ];
    let mut exit_angles = HashMap::new();
    //  exit_angles.insert(0, 90.0);

    let mut entry_angles = HashMap::new();
    // entry_angles.insert(1, 0.0);
    // entry_angles.insert(2, -90.0);
    let segments = hobby(
        &points,
        None,
        false,
        Some(&entry_angles),
        Some(&exit_angles),
    );
    for (i, seg) in segments.iter().enumerate() {
        println!("Segment {}: {}", i, seg);
    }
}
