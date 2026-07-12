use std::env;
use std::fs::File;
use std::time::Instant;

use map_preprocessor::build_map;
use map_preprocessor::HEADING_TOLERANCE_DEGREES;

/// Queries the spatial grid for the nearest road to the given point.
///
/// Arguments:
/// * `args[1]` - The command, which must be "--query".
/// * `args[2]` - The path to the binary spatial grid file.
/// * `args[3]` - A comma-separated string of longitude and latitude (e.g., "-122.4194,37.7749").
/// * `args[4]` - Optional heading in degrees (e.g., "45").
///
/// Panics if the arguments are invalid or if the file cannot be opened.
fn run_query(args: &[String]) {
    let bin_path = &args[2];
    let lon_lat: Vec<f64> = args[3].split(',').filter_map(|s| s.parse().ok()).collect();
    if lon_lat.len() != 2 {
        eprintln!("Usage: {} --query <roads.bin> <lon,lat> [heading]", args[0]);
        std::process::exit(1);
    }

    let mut car_heading: Option<f64> = None;
    if args.len() >= 5 {
        car_heading = args[4].parse().ok();
    }

    let start = Instant::now();
    let file = File::open(bin_path).expect("Failed to open bin file");
    let mmap = unsafe {
        memmap2::MmapOptions::new()
            .map(&file)
            .expect("Failed to map file")
    };
    println!("Mapped file in {:.2?}", start.elapsed());

    let start_access = Instant::now();
    let grid = rkyv::access::<common::ArchivedSpatialGrid, rkyv::rancor::Error>(&mmap)
        .expect("Failed to access spatial grid");
    println!("Accessed grid in {:.2?}", start_access.elapsed());

    let point = [lon_lat[0], lon_lat[1]];
    let start_query = Instant::now();

    if let Some(heading) = car_heading {
        println!(
            "Filtering for heading: {} degrees (+/- {} tolerance)",
            heading, HEADING_TOLERANCE_DEGREES
        );
    }

    let (speed_limit, dist) = grid.nearest_road(&point, car_heading, HEADING_TOLERANCE_DEGREES);

    if let (Some(limit), Some(d)) = (speed_limit, dist) {
        println!(
            "Nearest road speed limit: {} mph (distance: {:.5} deg)",
            limit, d
        );
    } else {
        println!("No valid roads found.");
    }

    println!("Query time: {:.2?}", start_query.elapsed());
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() >= 4 && args[1] == "--query" {
        run_query(&args);
        return;
    }

    if args.len() != 2 {
        eprintln!("Usage: {} <input.pbf>", args[0]);
        eprintln!("       {} --query <input.bin> <lon,lat> [heading]", args[0]);
        std::process::exit(1);
    }

    let input_path = std::path::Path::new(&args[1]);
    let file_stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("map");
    let base_name = file_stem.strip_suffix(".osm").unwrap_or(file_stem);

    // Determine the common directory (works whether run from workspace root or map-preprocessor dir)
    let common_dir = if std::path::Path::new("../common").exists() {
        "../common"
    } else {
        "common"
    };

    let state_bins_dir = format!("{}/state_bins", common_dir);
    if let Err(e) = std::fs::create_dir_all(&state_bins_dir) {
        eprintln!("Failed to create state_bins directory: {}", e);
    }

    let output_path_str = format!("{}/{}.bin", state_bins_dir, base_name);
    let output_path = std::path::Path::new(&output_path_str);

    if let Err(e) = build_map(input_path, output_path, |progress| {
        // Just print progress when run from CLI
        println!("Progress: {:.0}%", progress * 100.0);
    }) {
        eprintln!("Preprocessing failed: {}", e);
        std::process::exit(1);
    }
}
