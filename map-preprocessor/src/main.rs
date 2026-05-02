use std::collections::HashMap;
use std::env;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::Instant;

use common::{RoadData, RoadLine, RoadTree};
use osmpbfreader::{OsmObj, OsmPbfReader};
use rstar::primitives::GeomWithData;
use rstar::PointDistance;

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

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() >= 4 && args[1] == "--query" {
        let bin_path = &args[2];
        let lon_lat: Vec<f64> = args[3].split(',').filter_map(|s| s.parse().ok()).collect();
        if lon_lat.len() != 2 {
            eprintln!("Usage: {} --query <roads.bin> <lon,lat>", args[0]);
            std::process::exit(1);
        }

        let start = Instant::now();
        let file = File::open(bin_path).expect("Failed to open bin file");
        let mmap = unsafe { memmap2::MmapOptions::new().map(&file).expect("Failed to map file") };
        println!("Mapped file in {:.2?}", start.elapsed());

        let start_deser = Instant::now();
        let tree: RoadTree = bincode::deserialize(&mmap).expect("Failed to deserialize tree");
        println!("Deserialized tree in {:.2?}", start_deser.elapsed());

        let point = [lon_lat[0], lon_lat[1]];
        let start_query = Instant::now();
        if let Some(nearest) = tree.nearest_neighbor(&point) {
            let dist = nearest.distance_2(&point).sqrt();
            println!("Nearest road speed limit: {} mph (distance: {:.5} deg)", nearest.data.speed_limit_mph, dist);
        } else {
            println!("No roads found.");
        }
        println!("Query time: {:.2?}", start_query.elapsed());
        return;
    }

    if args.len() != 3 {
        eprintln!("Usage: {} <input.pbf> <output.bin>", args[0]);
        eprintln!("       {} --query <roads.bin> <lon,lat>", args[0]);
        std::process::exit(1);
    }

    let input_path = &args[1];
    let output_path = &args[2];

    println!("Parsing OSM PBF file: {}", input_path);
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

                    // First try to parse an explicit maxspeed tag
                    if let Some(maxspeed_str) = way.tags.get("maxspeed") {
                        final_speed = parse_speed_limit(maxspeed_str);
                    }

                    // If that failed or wasn't present, fall back to the default
                    if final_speed.is_none() {
                        final_speed = get_default_speed_limit(highway_type);
                    }

                    if let Some(speed_limit) = final_speed {
                        way_count += 1;
                        let data = RoadData {
                            speed_limit_mph: speed_limit,
                        };

                        for window in way.nodes.windows(2) {
                            let n1_id = window[0];
                            let n2_id = window[1];

                            if let (Some(&p1), Some(&p2)) = (nodes.get(&n1_id), nodes.get(&n2_id)) {
                                let line = RoadLine::new(p1, p2);
                                let segment = GeomWithData::new(line, data.clone());
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
    println!("Found {} roads with speed limits, yielding {} segments.", way_count, segment_count);

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
