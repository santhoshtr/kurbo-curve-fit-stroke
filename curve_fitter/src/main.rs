//! Command-line interface for curve fitting operations

use clap::Parser;
use std::fs;
use std::path::PathBuf;

use curve_fitter::TestRunner;

#[derive(Parser)]
#[command(name = "curve-fit")]
#[command(about = "Curve fitting with JSON-driven tests")]
#[command(
    long_about = "Execute curve fitting tests defined in JSON files. Each test specifies a sequence of operations (fit_curve, stroke, refit_stroke, register_skeleton, refit_with_skeleton) on a set of input points."
)]
struct Args {
    /// Run a single test from JSON file
    #[arg(short, long)]
    test: Option<PathBuf>,

    /// Run all tests in a directory
    #[arg(long)]
    test_dir: Option<PathBuf>,

    /// List all available tests
    #[arg(short, long)]
    list: bool,

    /// Enable G1 continuity validation output
    #[arg(long, default_value = "false")]
    validate: bool,

    /// Output directory for SVG files (default: ./outputs)
    #[arg(short, long, default_value = "outputs")]
    output: PathBuf,

    /// Run verbose output
    #[arg(short, long)]
    verbose: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Create output directory if it doesn't exist
    fs::create_dir_all(&args.output)?;

    let runner = TestRunner::new(args.verbose, !args.validate, args.output);

    match (args.test, args.test_dir, args.list) {
        (Some(test_file), None, false) => run_single_test(&runner, &test_file)?,
        (None, Some(test_dir), false) => run_all_tests(&runner, &test_dir)?,
        (None, None, true) => list_available_tests()?,
        (None, None, false) => {
            eprintln!("No test specified. Use --help for usage information.");
            std::process::exit(1);
        }
        _ => {
            eprintln!("Error: Specify only one of --test, --test-dir, or --list");
            std::process::exit(1);
        }
    }

    Ok(())
}

fn run_single_test(
    runner: &TestRunner,
    path: &std::path::Path,
) -> Result<(), Box<dyn std::error::Error>> {
    if !path.exists() {
        eprintln!("Test file not found: {}", path.display());
        std::process::exit(1);
    }

    let test_case = curve_fitter::TestCase::from_file(path)?;
    runner.run_test(&test_case).map_err(|e| e.into())
}

fn run_all_tests(runner: &TestRunner, dir: &PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    if !dir.is_dir() {
        eprintln!("Test directory not found: {}", dir.display());
        std::process::exit(1);
    }

    let mut test_count = 0;
    let mut failed_count = 0;

    println!("Scanning {} for JSON test files...", dir.display());

    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();

        if path.extension().is_some_and(|ext| ext == "json") {
            test_count += 1;

            match curve_fitter::TestCase::from_file(&path) {
                Ok(test_case) => {
                    if let Err(e) = runner.run_test(&test_case) {
                        println!("✗ Test FAILED: {}", e);
                        failed_count += 1;
                    }
                }
                Err(e) => {
                    println!("✗ Failed to load test from {}: {}", path.display(), e);
                    failed_count += 1;
                }
            }
        }
    }

    println!("\n=== Test Summary ===");
    println!(
        "Total: {}, Passed: {}, Failed: {}",
        test_count,
        test_count - failed_count,
        failed_count
    );

    if failed_count > 0 {
        std::process::exit(1);
    }

    Ok(())
}

fn list_available_tests() -> Result<(), Box<dyn std::error::Error>> {
    // Try multiple paths
    let test_dirs = vec![
        PathBuf::from("tests"),
        PathBuf::from("curve_fitter/tests"),
        PathBuf::from("./curve_fitter/tests"),
    ];

    let mut test_dir = None;
    for dir in &test_dirs {
        if dir.exists() && dir.is_dir() {
            test_dir = Some(dir.clone());
            break;
        }
    }

    let test_dir =
        test_dir.ok_or("No tests directory found. Try running from the workspace root.")?;

    println!("Available tests in {}:", test_dir.display());

    let mut count = 0;
    for entry in fs::read_dir(&test_dir)? {
        let entry = entry?;
        let path = entry.path();

        if path.extension().is_some_and(|ext| ext == "json")
            && let Ok(test_case) = curve_fitter::TestCase::from_file(&path)
        {
            count += 1;
            println!("  • {} - {}", test_case.name, test_case.description);
        }
    }

    if count == 0 {
        println!("  (No JSON test files found)");
    }

    Ok(())
}
