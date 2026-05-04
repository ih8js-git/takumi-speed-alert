use common::{RoadTree, angle_diff};
use measurements::Speed;
use rstar::PointDistance;
use serde_json::Value;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

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

struct Recorder {
    start_time: Option<u64>,
    end_time: Option<u64>,
    file: Option<File>,
    temp_path: String,
    last_point_time: Option<Instant>,
}

impl Recorder {
    fn new() -> Self {
        let logs_dir = std::path::Path::new("common/car_logs");
        if !logs_dir.exists() {
            std::fs::create_dir_all(logs_dir).expect("Failed to create car_logs directory");
        }
        Self {
            start_time: None,
            end_time: None,
            file: None,
            temp_path: String::new(),
            last_point_time: None,
        }
    }

    fn record_point(
        &mut self,
        timestamp: &str,
        lon: f64,
        lat: f64,
        track: Option<f64>,
        speed: f64,
    ) {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let track_str = match track {
            Some(t) => t.to_string(),
            None => "".to_string(),
        };

        if self.start_time.is_none() {
            self.start_time = Some(now);
            self.temp_path = format!("common/car_logs/{}_inprogress.csv", now);
            let mut f = File::create(&self.temp_path).expect("Failed to create record file");
            writeln!(f, "time,long,lat,track,speed").unwrap();
            writeln!(f, "{},{},{},{},{}", timestamp, lon, lat, track_str, speed).unwrap();
            self.file = Some(f);
            self.last_point_time = Some(Instant::now());
        } else {
            if let Some(last_time) = self.last_point_time {
                let delta = last_time.elapsed().as_secs_f64();
                self.last_point_time = Some(Instant::now());
                if let Some(f) = &mut self.file {
                    writeln!(f, "{:.3},{},{},{},{}", delta, lon, lat, track_str, speed).unwrap();
                }
            }
        }
        self.end_time = Some(now);
    }

    fn finish(mut self) {
        if let (Some(f), Some(start), Some(end)) =
            (self.file.take(), self.start_time, self.end_time)
        {
            drop(f); // ensure it's flushed/closed
            let new_path = format!("common/car_logs/{}-{}.csv", start, end);
            let _ = std::fs::rename(&self.temp_path, new_path);
        }
    }
}

fn main() {
    let mut args: Vec<String> = env::args().collect();
    let record_flag_idx = args.iter().position(|a| a == "--record");
    let record = record_flag_idx.is_some();
    if let Some(idx) = record_flag_idx {
        args.remove(idx);
    }

    if args.len() != 1 {
        eprintln!("Usage: {} [--record]", args[0]);
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

    let bin_dir = std::path::Path::new("common/state_bins");
    if !bin_dir.exists() || !bin_dir.is_dir() {
        eprintln!("Error: {} directory not found.", bin_dir.display());
        std::process::exit(1);
    }

    let mut bin_files = Vec::new();
    for entry in std::fs::read_dir(bin_dir).expect("Failed to read state_bins directory") {
        if let Ok(entry) = entry {
            let path = entry.path();
            if path.is_file() && path.extension().and_then(|s| s.to_str()) == Some("bin") {
                bin_files.push(path);
            }
        }
    }

    if bin_files.is_empty() {
        eprintln!("Error: No state bin files found in {}.", bin_dir.display());
        std::process::exit(1);
    } else if bin_files.len() > 1 {
        eprintln!(
            "Error: Multiple state bin files found in {}. Loading multiple states will be implemented later.",
            bin_dir.display()
        );
        std::process::exit(1);
    }

    let map_path = bin_files[0].to_str().unwrap().to_string();

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

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nShutting down gracefully...");
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    stream
        .set_read_timeout(Some(std::time::Duration::from_millis(500)))
        .expect("Failed to set read timeout");

    let mut speeding_start_time: Option<Instant> = None;
    let mut recorder = if record { Some(Recorder::new()) } else { None };

    // Read the stream line by line
    while running.load(Ordering::SeqCst) {
        match reader.read_line(&mut line) {
            Ok(0) => break, // EOF or stream closed
            Ok(_) => {
                process_gps_line(
                    &line,
                    &tree,
                    &mut speeding_start_time,
                    &config,
                    &mut recorder,
                );
                line.clear();
            }
            Err(e) => {
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut
                {
                    continue;
                }
                break;
            }
        }
    }

    if let Some(r) = recorder {
        r.finish();
    }
}

fn process_gps_line(
    line: &str,
    tree: &RoadTree,
    speeding_start_time: &mut Option<Instant>,
    config: &Config,
    recorder: &mut Option<Recorder>,
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
    let time_str = json["time"].as_str().unwrap_or("");

    if let Some(r) = recorder {
        r.record_point(time_str, lon, lat, track, speed);
    }

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
