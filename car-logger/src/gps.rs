use crate::config::Config;
use crate::recorder::Recorder;
use common::{RoadTree, angle_diff};
use measurements::Speed;
use rstar::PointDistance;
use serde_json::Value;
use std::time::Instant;

pub fn process_gps_line(
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
