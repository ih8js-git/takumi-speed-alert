use common::{RoadTree, angle_diff};
use measurements::Speed;
use rstar::PointDistance;
use serde_json::Value;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::Instant;

use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
struct Config {
    heading_tolerance_degrees: f64,
    speed_limit_tolerance_mph: f64,
    speeding_duration_before_beep_seconds: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            heading_tolerance_degrees: 45.0,
            speed_limit_tolerance_mph: 10.0,
            speeding_duration_before_beep_seconds: 5.0,
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("Usage: {} <map.bin>", args[0]);
        std::process::exit(1);
    }

    let config_path = "config.json";
    let config: Config = if let Ok(config_str) = std::fs::read_to_string(config_path) {
        serde_json::from_str(&config_str).unwrap_or_else(|e| {
            eprintln!("Failed to parse {}: {}", config_path, e);
            Config::default()
        })
    } else {
        println!("{} not found, using default configuration.", config_path);
        let default_config = Config::default();
        if let Ok(config_str) = serde_json::to_string_pretty(&default_config) {
            let _ = std::fs::write(config_path, config_str);
        }
        default_config
    };

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
        process_gps_line(&line, &tree, &mut speeding_start_time, &config);
        line.clear();
    }
}

fn process_gps_line(
    line: &str,
    tree: &RoadTree,
    speeding_start_time: &mut Option<Instant>,
    config: &Config,
) {
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

    let (speed_limit_mph, matched_dist) = match_road(&point, track, tree, config);

    print!(
        "Fix: {}D | Lat: {:.6} | Lon: {:.6} | Speed: {:.1} mph",
        mode, lat, lon, speed_mph
    );

    if let Some(limit) = speed_limit_mph {
        print!(
            " | Limit: {} mph (dist: {:.5} deg)",
            limit,
            matched_dist.unwrap()
        );

        if speed_mph > (limit as f64 + config.speed_limit_tolerance_mph) {
            if speeding_start_time.is_none() {
                *speeding_start_time = Some(Instant::now());
            } else if let Some(start) = speeding_start_time {
                if start.elapsed().as_secs_f64() >= config.speeding_duration_before_beep_seconds {
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

fn match_road(
    point: &[f64; 2],
    track: Option<f64>,
    tree: &RoadTree,
    config: &Config,
) -> (Option<u8>, Option<f64>) {
    if let Some(heading) = track {
        for nearest in tree.nearest_neighbor_iter(point) {
            let dist = nearest.distance_2(point).sqrt();
            if dist > 0.05 {
                break;
            }

            let road_bearing = nearest.data.bearing as f64;
            let diff = angle_diff(heading, road_bearing);
            let mut match_found = diff <= config.heading_tolerance_degrees;

            if !match_found && !nearest.data.is_one_way {
                let reverse_diff = angle_diff(heading, road_bearing + 180.0);
                if reverse_diff <= config.heading_tolerance_degrees {
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
