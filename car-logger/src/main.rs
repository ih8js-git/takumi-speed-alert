mod config;
mod gps;
mod recorder;
use common::ArchivedSpatialGrid;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;

use config::{CONFIG_PATH, Config};
use gps::process_gps_line;
use recorder::Recorder;

fn main() {
    std::thread::spawn(|| {
        Recorder::cleanup_previous_runs();
    });

    let mut args: Vec<String> = env::args().collect();
    let record_flag_idx = args.iter().position(|a| a == "--record");
    let record = record_flag_idx.is_some();
    if let Some(idx) = record_flag_idx {
        args.remove(idx);
    }

    let mut replay_file = None;
    if let Some(idx) = args.iter().position(|a| a == "--replay") {
        if idx + 1 < args.len() {
            replay_file = Some(args[idx + 1].clone());
            args.remove(idx + 1);
            args.remove(idx);
        } else {
            eprintln!("Error: --replay requires a file argument.");
            std::process::exit(1);
        }
    }

    if args.len() != 1 {
        eprintln!("Usage: {} [--record] [--replay <csv_file>]", args[0]);
        std::process::exit(1);
    }

    let config: Config = if let Ok(config_str) = std::fs::read_to_string(CONFIG_PATH) {
        serde_json::from_str(&config_str).unwrap_or_else(|e| {
            eprintln!("Failed to parse {}: {}", CONFIG_PATH, e);
            Config::default()
        })
    } else {
        println!("{} not found, using default configuration.", CONFIG_PATH);
        let default_config = Config::default();
        if let Ok(config_str) = serde_json::to_string_pretty(&default_config) {
            let _ = std::fs::write(CONFIG_PATH, config_str);
        }
        default_config
    };

    // Prioritize local common directory to completely isolate local development
    let bin_dirs = [
        std::path::Path::new("common/state_bins"),
        std::path::Path::new("/boot/firmware/state_bins"),
    ];

    let mut found_bin_dir = None;
    for dir in &bin_dirs {
        if dir.exists() && dir.is_dir() {
            // Check if there are any .bin files inside
            if let Ok(entries) = std::fs::read_dir(dir) {
                let has_bins = entries.filter_map(Result::ok).any(|e| {
                    e.path().is_file()
                        && e.path().extension().and_then(|s| s.to_str()) == Some("bin")
                });
                if has_bins {
                    found_bin_dir = Some(*dir);
                    break;
                }
            }
        }
    }

    let Some(bin_dir) = found_bin_dir else {
        eprintln!("Error: No state .bin files found. Please place your .bin map file in one of:");
        for dir in &bin_dirs {
            eprintln!("  - {}", dir.display());
        }
        std::process::exit(1);
    };

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

    println!("Loading spatial grid from {}...", map_path);
    let start_load = Instant::now();

    let file = File::open(&map_path).expect("Failed to open map.bin");
    let mmap = unsafe {
        memmap2::MmapOptions::new()
            .map(&file)
            .expect("Failed to map file")
    };
    let grid = rkyv::access::<ArchivedSpatialGrid, rkyv::rancor::Error>(&mmap)
        .expect("Failed to access spatial grid");
    println!(
        "Loaded spatial grid in {:.2?} (zero-copy from mmap)",
        start_load.elapsed()
    );

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nShutting down gracefully...");
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    let mut speeding_start_time: Option<Instant> = None;
    let mut recorder = if record { Some(Recorder::new()) } else { None };

    if let Some(file_path) = replay_file {
        println!("Replaying from {}...", file_path);
        let file = File::open(&file_path).expect("Failed to open replay file");
        let reader = BufReader::new(file);
        let mut lines = reader.lines();

        let _ = lines.next(); // Skip header
        let mut first_row = true;

        for line_res in lines {
            if !running.load(Ordering::SeqCst) {
                break;
            }
            if let Ok(line) = line_res {
                let parts: Vec<&str> = line.split(',').collect();
                if parts.len() < 5 {
                    continue;
                }
                let time_str = parts[0];
                let lon = parts[1];
                let lat = parts[2];
                let track = parts[3];
                let speed = parts[4];

                if first_row {
                    first_row = false;
                } else if let Ok(delta) = time_str.parse::<f64>() {
                    std::thread::sleep(std::time::Duration::from_secs_f64(delta));
                }

                let track_json = if track.is_empty() {
                    "null".to_string()
                } else {
                    track.to_string()
                };

                let fake_json = format!(
                    r#"{{"class":"TPV","mode":3,"lat":{},"lon":{},"speed":{},"track":{},"time":"{}"}}"#,
                    lat, lon, speed, track_json, time_str
                );

                process_gps_line(
                    &fake_json,
                    grid,
                    &mut speeding_start_time,
                    &config,
                    &mut recorder,
                );
            }
        }
    } else {
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

        stream
            .set_read_timeout(Some(std::time::Duration::from_millis(500)))
            .expect("Failed to set read timeout");

        // Read the stream line by line
        while running.load(Ordering::SeqCst) {
            match reader.read_line(&mut line) {
                Ok(0) => break, // EOF or stream closed
                Ok(_) => {
                    process_gps_line(
                        &line,
                        grid,
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
    }

    if let Some(r) = recorder {
        r.finish();
    }
}
