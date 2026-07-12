use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;
use std::time::Instant;

use common::{GridRoadSegment, SpatialGrid};
use measurements::Speed;
use osmpbfreader::{OsmObj, OsmPbfReader};
use rustc_hash::FxHashMap;

pub const HEADING_TOLERANCE_DEGREES: f64 = 45.0;

fn parse_speed_limit(val: &str) -> Option<u8> {
    let val_lower = val.to_lowercase();
    let num_str: String = val_lower.chars().filter(|c| c.is_digit(10)).collect();
    if num_str.is_empty() {
        return None;
    }
    let speed: u8 = num_str.parse().ok()?;

    if val_lower.contains("km/h") || val_lower.contains("kmh") {
        Some(
            Speed::from_kilometers_per_hour(speed as f64)
                .as_miles_per_hour()
                .round() as u8,
        )
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
        _ => None,
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

struct ProgressReader<R: Read, F: FnMut(f32)> {
    inner: R,
    bytes_read: u64,
    total_bytes: u64,
    last_update: Instant,
    callback: F,
}

impl<R: Read, F: FnMut(f32)> ProgressReader<R, F> {
    fn new(inner: R, total_bytes: u64, callback: F) -> Self {
        Self {
            inner,
            bytes_read: 0,
            total_bytes,
            last_update: Instant::now(),
            callback,
        }
    }
}

impl<R: Read, F: FnMut(f32)> Read for ProgressReader<R, F> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.bytes_read += n as u64;
        
        if self.last_update.elapsed().as_millis() > 100 {
            self.last_update = Instant::now();
            if self.total_bytes > 0 {
                // Ensure we don't go over 1.0 (100%)
                let progress = (self.bytes_read as f64 / self.total_bytes as f64).min(1.0) as f32;
                // Leave 10% for grid building and serialization
                (self.callback)(progress * 0.90);
            }
        }
        
        Ok(n)
    }
}

pub fn build_map<F: FnMut(f32)>(input_path: &Path, output_path: &Path, mut progress_callback: F) -> Result<(), String> {
    println!("Parsing OSM PBF file: {}", input_path.display());
    let start_time = Instant::now();

    let file = File::open(input_path).map_err(|e| format!("Failed to open input file: {}", e))?;
    let total_bytes = file.metadata().map(|m| m.len()).unwrap_or(0);
    
    let reader = ProgressReader::new(file, total_bytes, &mut progress_callback);
    let mut pbf = OsmPbfReader::new(reader);

    // Swap HashMap for FxHashMap for massive performance increase
    let mut nodes = FxHashMap::default();
    let mut segments: Vec<GridRoadSegment> = Vec::new();

    let mut way_count = 0;
    let mut segment_count = 0;

    for obj in pbf.iter().filter_map(Result::ok) {
        match obj {
            OsmObj::Node(node) => {
                nodes.insert(node.id.0, [node.lon(), node.lat()]);
            }
            OsmObj::Way(way) => {
                let Some(highway_type) = way.tags.get("highway") else {
                    continue;
                };

                let final_speed = way
                    .tags
                    .get("maxspeed")
                    .and_then(|s| parse_speed_limit(s))
                    .or_else(|| get_default_speed_limit(highway_type));

                let Some(speed_limit) = final_speed else {
                    continue;
                };

                let oneway_tag = way.tags.get("oneway").map(|s| s.as_str()).unwrap_or("no");
                let is_reversed = oneway_tag == "-1";
                let is_one_way = oneway_tag == "yes"
                    || oneway_tag == "true"
                    || oneway_tag == "1"
                    || is_reversed
                    || highway_type == "motorway"
                    || highway_type == "motorway_link";

                way_count += 1;

                for window in way.nodes.windows(2) {
                    let (n1_id, n2_id) = (window[0].0, window[1].0);

                    let (Some(&p1), Some(&p2)) = (nodes.get(&n1_id), nodes.get(&n2_id)) else {
                        continue;
                    };

                    let mut bearing = calculate_bearing(p1[0], p1[1], p2[0], p2[1]);
                    if is_reversed {
                        bearing = (bearing + 180) % 360;
                    }

                    segments.push(GridRoadSegment {
                        p1,
                        p2,
                        speed_limit_mph: speed_limit,
                        is_one_way,
                        bearing,
                    });
                    segment_count += 1;
                }
            }
            _ => {}
        }
    }

    println!("Parsed in {:.2?}", start_time.elapsed());
    println!("Found {} roads with speed limits, yielding {} segments.", way_count, segment_count);

    progress_callback(0.95);

    let grid_start = Instant::now();
    println!("Building spatial grid (cell size: {:.4}°)...", SpatialGrid::DEFAULT_CELL_SIZE);
    let grid = SpatialGrid::from_segments(&segments, SpatialGrid::DEFAULT_CELL_SIZE);
    println!("Grid built in {:.2?}", grid_start.elapsed());

    let save_start = Instant::now();
    println!("Serializing spatial grid to {}...", output_path.display());
    let bytes = rkyv::to_bytes::<rkyv::rancor::Error>(&grid).map_err(|e| format!("Failed to serialize grid: {}", e))?;
    let mut out_file = File::create(output_path).map_err(|e| format!("Failed to create output file: {}", e))?;
    out_file.write_all(&bytes).map_err(|e| format!("Failed to write grid: {}", e))?;
    println!("Serialized in {:.2?}", save_start.elapsed());

    progress_callback(1.0);
    println!("Map preprocessing complete!");
    
    Ok(())
}
