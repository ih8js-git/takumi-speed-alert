use common::{RoadTree, angle_diff};
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
// How much over the speed limit you can go before triggering an alert.
const SPEED_LIMIT_TOLERANCE_MPH: f64 = 5.0;
// How long you must be speeding continuously before the beep sounds.
const SPEEDING_DURATION_BEFORE_BEEP_SECONDS: f64 = 5.0;

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

    let mut speeding_start_time: Option<Instant> = None;

    // Read the stream line by line
    while let Ok(bytes_read) = reader.read_line(&mut line) {
        if bytes_read == 0 {
            break; // EOF or stream closed
        }
        process_gps_line(&line, &tree, &mut speeding_start_time);
        line.clear();
    }
}

fn process_gps_line(line: &str, tree: &RoadTree, speeding_start_time: &mut Option<Instant>) {
    let Ok(json) = serde_json::from_str::<Value>(line) else {
        return;
    };

    // Filter for TPV (Time-Position-Velocity) packets
    if json["class"] != "TPV" {
        return;
    }

    let mode = json["mode"].as_i64().unwrap_or(0);

    // mode 2 is a 2D fix, mode 3 is a 3D fix
    if mode < 2 {
        println!("Waiting for GPS fix...");
        *speeding_start_time = None;
        return;
    }

    let lat = json["lat"].as_f64().unwrap_or(0.0);
    let lon = json["lon"].as_f64().unwrap_or(0.0);
    let speed = json["speed"].as_f64().unwrap_or(0.0); // m/s
    let speed_mph = Speed::from_meters_per_second(speed).as_miles_per_hour();
    let track = json["track"].as_f64(); // True course in degrees

    let point = [lon, lat];

    let (speed_limit_mph, matched_dist) = match_road(&point, track, tree);

    print!(
        "Fix: {}D | Lat: {:.6} | Lon: {:.6} | Speed: {:.1} mph",
        mode,
        lat,
        lon,
        speed_mph
    );

    if let Some(limit) = speed_limit_mph {
        print!(
            " | Limit: {} mph (dist: {:.5} deg)",
            limit,
            matched_dist.unwrap()
        );

        if speed_mph > (limit as f64 + SPEED_LIMIT_TOLERANCE_MPH) {
            if speeding_start_time.is_none() {
                *speeding_start_time = Some(Instant::now());
            } else if let Some(start) = speeding_start_time {
                if start.elapsed().as_secs_f64() >= SPEEDING_DURATION_BEFORE_BEEP_SECONDS {
                    print!(" | *** BEEP BEEP BEEP! ***");
                }
            }
        } else {
            *speeding_start_time = None;
        }
    } else {
        print!(" | Limit: Unknown");
        *speeding_start_time = None;
    }

    if let Some(h) = track {
        print!(" | Heading: {:.1}°", h);
    }
    println!();
}

fn match_road(point: &[f64; 2], track: Option<f64>, tree: &RoadTree) -> (Option<u8>, Option<f64>) {
    if let Some(heading) = track {
        for nearest in tree.nearest_neighbor_iter(point) {
            let dist = nearest.distance_2(point).sqrt();
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
                return (Some(nearest.data.speed_limit_mph), Some(dist));
            }
        }
    } else {
        // Fallback if no track/heading is provided by GPS yet
        if let Some(nearest) = tree.nearest_neighbor(point) {
            return (
                Some(nearest.data.speed_limit_mph),
                Some(nearest.distance_2(point).sqrt()),
            );
        }
    }

    (None, None)
}
