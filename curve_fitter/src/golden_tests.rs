//! Golden geometry tests
//!
//! Every JSON test case is executed and each path-producing stage is compared
//! coordinate-by-coordinate against a stored snapshot in `tests/golden/`.
//! Unlike the ok/err assertions elsewhere, these catch changes to the actual
//! geometry the algorithms produce.
//!
//! After an intentional geometry change, regenerate the snapshots with:
//!
//! ```text
//! UPDATE_GOLDEN=1 cargo test -p curve_fitter golden
//! ```
//!
//! and review the diff of `tests/golden/` alongside the SVG outputs.

use std::fs;
use std::path::PathBuf;

use kurbo::{BezPath, PathEl, Point};

use crate::test_runner::execute_operations;
use crate::test_schema::TestCase;

const COORD_TOLERANCE: f64 = 1e-6;

#[test]
fn golden_geometry() {
    let tests_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests");
    let golden_dir = tests_dir.join("golden");
    let update = std::env::var("UPDATE_GOLDEN").is_ok();

    if update {
        fs::create_dir_all(&golden_dir).unwrap();
    }

    let mut case_files: Vec<PathBuf> = fs::read_dir(&tests_dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "json"))
        .collect();
    case_files.sort();
    assert!(!case_files.is_empty(), "no JSON test cases found");

    let mut compared = 0;
    for case_file in case_files {
        let case = TestCase::from_file(&case_file)
            .unwrap_or_else(|e| panic!("{}: {}", case_file.display(), e));
        let stages = execute_operations(&case).unwrap_or_else(|e| panic!("{}: {}", case.name, e));

        for stage in stages {
            let Some(path) = stage.path else { continue };
            let golden_file = golden_dir.join(format!("{}--{}.path", case.name, stage.operation));

            if update {
                fs::write(&golden_file, path.to_svg()).unwrap();
                continue;
            }

            let golden_svg = fs::read_to_string(&golden_file).unwrap_or_else(|_| {
                panic!(
                    "missing golden snapshot {} — generate with UPDATE_GOLDEN=1 cargo test",
                    golden_file.display()
                )
            });
            let golden = BezPath::from_svg(&golden_svg).unwrap();
            assert_path_matches(&case.name, &stage.operation, &golden, &path);
            compared += 1;
        }
    }

    assert!(update || compared > 0, "no golden snapshots compared");
}

/// Compare two paths element-by-element within COORD_TOLERANCE
fn assert_path_matches(case: &str, operation: &str, golden: &BezPath, actual: &BezPath) {
    let golden_els = golden.elements();
    let actual_els = actual.elements();
    assert_eq!(
        golden_els.len(),
        actual_els.len(),
        "{case}/{operation}: {} elements, golden has {}",
        actual_els.len(),
        golden_els.len()
    );

    for (i, (g, a)) in golden_els.iter().zip(actual_els.iter()).enumerate() {
        let (g_pts, a_pts): (Vec<Point>, Vec<Point>) = match (g, a) {
            (PathEl::MoveTo(g1), PathEl::MoveTo(a1)) => (vec![*g1], vec![*a1]),
            (PathEl::LineTo(g1), PathEl::LineTo(a1)) => (vec![*g1], vec![*a1]),
            (PathEl::QuadTo(g1, g2), PathEl::QuadTo(a1, a2)) => {
                (vec![*g1, *g2], vec![*a1, *a2])
            }
            (PathEl::CurveTo(g1, g2, g3), PathEl::CurveTo(a1, a2, a3)) => {
                (vec![*g1, *g2, *g3], vec![*a1, *a2, *a3])
            }
            (PathEl::ClosePath, PathEl::ClosePath) => (Vec::new(), Vec::new()),
            _ => panic!("{case}/{operation}: element {i} kind mismatch: golden {g:?}, actual {a:?}"),
        };

        for (g_pt, a_pt) in g_pts.iter().zip(&a_pts) {
            let deviation = (*a_pt - *g_pt).hypot();
            assert!(
                deviation <= COORD_TOLERANCE,
                "{case}/{operation}: element {i} deviates by {deviation:.3e}: \
                 golden {g_pt:?}, actual {a_pt:?}"
            );
        }
    }
}
