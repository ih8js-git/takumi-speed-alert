use std::collections::HashMap;
use std::env;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::Instant;

use common::{RoadData, RoadLine, RoadTree};
use osmpbfreader::{OsmObj, OsmPbfReader};
use rstar::PointDistance;
use rstar::primitives::GeomWithData;

// --- CONFIGURATION ---
// Easy to access variable for adjusting how strict our heading filter is.
// A tolerance of 45.0 means the car must be traveling within +/- 45 degrees of the road's direction.
const HEADING_TOLERANCE_DEGREES: f64 = 45.0;

fn parse_speed_limit(val: &str) -> Option<u8> {
    let s = val.to_lowercase();
    let num_str: String = s.chars().filter(|c| c.is_digit(10)).collect();
    if num_str.is_empty() {
        return None;
    }
    let speed: u8 = num_str.parse().ok()?;

    if s.contains("km/h") || s.contains("kmh") {
        Some((speed as f32 * 0.621371).round() as u8)
    } else {
        Some(speed) // Assume mph
    }
}

fn get_default_speed_limit(highway_type: &str) -> Option<u8> {
    match highway_type {
        "motorway" => Some(70),
        "motorway_link" => Some(45),
        "trunk" | "trunk_link" => Some(65),
        "primary" | "primary_link" => Some(55),
        "secondary" | "secondary_link" => Some(45),
        "tertiary" | "tertiary_link" => Some(45),
        "unclassified" => Some(35),
        "residential" => Some(25),
        "living_street" => Some(15),
        _ => None, // Things like footway, cycleway, path, etc. have no default speed limit
    }
}

fn calculate_bearing(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> u16 {
    let dy = lat2 - lat1;
    let lat_rad = lat1 * std::f64::consts::PI / 180.0;
    let dx = (lon2 - lon1) * lat_rad.cos();

    let math_angle = dy.atan2(dx) * 180.0 / std::f64::consts::PI;
    let mut compass = 90.0 - math_angle;
    while compass < 0.0 {
        compass += 360.0;
    }
    (compass.round() as u16) % 360
}

fn angle_diff(a: f64, b: f64) -> f64 {
    let diff = (a - b).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() >= 4 && args[1] == "--query" {
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

        let start_deser = Instant::now();
        let tree: RoadTree = bincode::deserialize(&mmap).expect("Failed to deserialize tree");
        println!("Deserialized tree in {:.2?}", start_deser.elapsed());

        let point = [lon_lat[0], lon_lat[1]];
        let start_query = Instant::now();

        let mut best_match = None;

        if let Some(heading) = car_heading {
            println!(
                "Filtering for heading: {} degrees (+/- {} tolerance)",
                heading, HEADING_TOLERANCE_DEGREES
            );
            for nearest in tree.nearest_neighbor_iter(&point) {
                let dist = nearest.distance_2(&point).sqrt();
                if dist > 0.05 {
                    // Stop searching if roads are too far
                    break;
                }

                let road_bearing = nearest.data.bearing as f64;
                let diff = angle_diff(heading, road_bearing);
                let mut match_found = diff <= HEADING_TOLERANCE_DEGREES;

                if !match_found && !nearest.data.is_one_way {
                    let reverse_diff = angle_diff(heading, road_bearing + 180.0);
                    if reverse_diff <= HEADING_TOLERANCE_DEGREES {
                        match_found = true;
                    }
                }

                if match_found {
                    best_match = Some((nearest, dist));
                    break;
                }
            }
        } else {
            if let Some(nearest) = tree.nearest_neighbor(&point) {
                let dist = nearest.distance_2(&point).sqrt();
                best_match = Some((nearest, dist));
            }
        }

        if let Some((nearest, dist)) = best_match {
            println!(
                "Nearest road speed limit: {} mph (distance: {:.5} deg)",
                nearest.data.speed_limit_mph, dist
            );
            println!(
                "Road directionality: one_way={}, bearing={}",
                nearest.data.is_one_way, nearest.data.bearing
            );
        } else {
            println!("No valid roads found.");
        }

        println!("Query time: {:.2?}", start_query.elapsed());
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

    let output_path = format!("{}/{}.bin", common_dir, base_name);

    println!("Parsing OSM PBF file: {}", input_path.display());
    let start_time = Instant::now();

    let file = File::open(input_path).expect("Failed to open input file");
    let mut pbf = OsmPbfReader::new(file);

    let mut nodes = HashMap::new();
    let mut segments = Vec::new();

    let mut way_count = 0;
    let mut segment_count = 0;

    for obj in pbf.iter().filter_map(Result::ok) {
        match obj {
            OsmObj::Node(node) => {
                nodes.insert(node.id, [node.lon(), node.lat()]);
            }
            OsmObj::Way(way) => {
                if let Some(highway_type) = way.tags.get("highway") {
                    let mut final_speed = None;

                    if let Some(maxspeed_str) = way.tags.get("maxspeed") {
                        final_speed = parse_speed_limit(maxspeed_str);
                    }

                    if final_speed.is_none() {
                        final_speed = get_default_speed_limit(highway_type);
                    }

                    if let Some(speed_limit) = final_speed {
                        let oneway_tag = way.tags.get("oneway").map(|s| s.as_str()).unwrap_or("no");
                        let mut is_one_way = oneway_tag == "yes"
                            || oneway_tag == "true"
                            || oneway_tag == "1"
                            || oneway_tag == "-1";
                        let is_reversed = oneway_tag == "-1";

                        if highway_type == "motorway" || highway_type == "motorway_link" {
                            is_one_way = true;
                        }

                        way_count += 1;

                        for window in way.nodes.windows(2) {
                            let n1_id = window[0];
                            let n2_id = window[1];

                            if let (Some(&p1), Some(&p2)) = (nodes.get(&n1_id), nodes.get(&n2_id)) {
                                let mut bearing = calculate_bearing(p1[0], p1[1], p2[0], p2[1]);
                                if is_reversed {
                                    bearing = (bearing + 180) % 360;
                                }

                                let data = RoadData {
                                    speed_limit_mph: speed_limit,
                                    is_one_way,
                                    bearing,
                                };

                                let line = RoadLine::new(p1, p2);
                                let segment = GeomWithData::new(line, data);
                                segments.push(segment);
                                segment_count += 1;
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }

    println!("Parsed in {:.2?}", start_time.elapsed());
    println!(
        "Found {} roads with speed limits, yielding {} segments.",
        way_count, segment_count
    );

    let tree_start = Instant::now();
    println!("Building R-Tree...");
    let tree = RoadTree::bulk_load(segments);
    println!("R-Tree built in {:.2?}", tree_start.elapsed());

    let save_start = Instant::now();
    println!("Serializing R-Tree to {}...", output_path);
    let out_file = File::create(output_path).expect("Failed to create output file");
    let mut writer = BufWriter::new(out_file);
    bincode::serialize_into(&mut writer, &tree).expect("Failed to serialize R-Tree");
    writer.flush().unwrap();
    println!("Serialized in {:.2?}", save_start.elapsed());

    println!("Map preprocessing complete!");
}
