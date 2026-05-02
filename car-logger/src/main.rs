use common::RoadTree;
use measurements::Speed;
use rstar::PointDistance;
use serde_json::Value;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::Instant;

// --- CONFIGURATION ---
// Tolerance for matching car's heading to road's bearing.
const HEADING_TOLERANCE_DEGREES: f64 = 45.0;

fn angle_diff(a: f64, b: f64) -> f64 {
    let diff = (a - b).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("Usage: {} <map.bin>", args[0]);
        std::process::exit(1);
    }

    let map_path = &args[1];

    println!("Loading spatial index from {}...", map_path);
    let start_load = Instant::now();

    let file = File::open(map_path).expect("Failed to open map.bin");
    let mmap = unsafe {
        memmap2::MmapOptions::new()
            .map(&file)
            .expect("Failed to map file")
    };
    let tree: RoadTree = bincode::deserialize(&mmap).expect("Failed to deserialize tree");
    println!("Loaded map index in {:.2?}", start_load.elapsed());

    // Connect to the local gpsd TCP socket
    let stream = TcpStream::connect("127.0.0.1:2947").expect("Failed to connect to gpsd");
    let mut reader = BufReader::new(&stream);
    let mut writer = &stream;

    // Send the WATCH command to tell gpsd to start streaming JSON
    writer
        .write_all(b"?WATCH={\"enable\":true,\"json\":true}\n")
        .expect("Failed to send WATCH command");

    let mut line = String::new();
    println!("Listening for gpsd data...");

    // Read the stream line by line
    while reader.read_line(&mut line).is_ok() {
        if let Ok(json) = serde_json::from_str::<Value>(&line) {
            // Filter for TPV (Time-Position-Velocity) packets
            if json["class"] == "TPV" {
                let mode = json["mode"].as_i64().unwrap_or(0);

                // mode 2 is a 2D fix, mode 3 is a 3D fix
                if mode >= 2 {
                    let lat = json["lat"].as_f64().unwrap_or(0.0);
                    let lon = json["lon"].as_f64().unwrap_or(0.0);
                    let speed = json["speed"].as_f64().unwrap_or(0.0); // m/s
                    let track = json["track"].as_f64(); // True course in degrees

                    let mut speed_limit_mph = None;
                    let mut matched_dist = None;

                    let point = [lon, lat];

                    if let Some(heading) = track {
                        for nearest in tree.nearest_neighbor_iter(&point) {
                            let dist = nearest.distance_2(&point).sqrt();
                            if dist > 0.05 {
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
                                speed_limit_mph = Some(nearest.data.speed_limit_mph);
                                matched_dist = Some(dist);
                                break;
                            }
                        }
                    } else {
                        // Fallback if no track/heading is provided by GPS yet
                        if let Some(nearest) = tree.nearest_neighbor(&point) {
                            speed_limit_mph = Some(nearest.data.speed_limit_mph);
                            matched_dist = Some(nearest.distance_2(&point).sqrt());
                        }
                    }

                    print!(
                        "Fix: {}D | Lat: {:.6} | Lon: {:.6} | Speed: {:.1} mph",
                        mode,
                        lat,
                        lon,
                        Speed::from_meters_per_second(speed).as_miles_per_hour()
                    );

                    if let Some(limit) = speed_limit_mph {
                        print!(
                            " | Limit: {} mph (dist: {:.5} deg)",
                            limit,
                            matched_dist.unwrap()
                        );
                    } else {
                        print!(" | Limit: Unknown");
                    }

                    if let Some(h) = track {
                        print!(" | Heading: {:.1}°", h);
                    }
                    println!();
                } else {
                    println!("Waiting for GPS fix...");
                }
            }
        }
        line.clear();
    }
}
