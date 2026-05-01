use serde_json::Value;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

fn main() {
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
                    let speed = json["speed"].as_f64().unwrap_or(0.0); // Speed in meters per second

                    println!(
                        "Fix: {}D | Lat: {:.6} | Lon: {:.6} | Speed: {:.2} m/s",
                        mode, lat, lon, speed
                    );
                } else {
                    println!("Waiting for GPS fix...");
                }
            }
        }
        line.clear();
    }
}
