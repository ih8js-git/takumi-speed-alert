use chrono::TimeZone;
use std::fs::File;
use std::io::Write;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

pub struct Recorder {
    start_time: Option<u64>,
    end_time: Option<u64>,
    file: Option<File>,
    temp_path: String,
    last_point_time: Option<Instant>,
    log_dir: String,
}

impl Recorder {
    pub fn get_log_dir() -> String {
        if std::path::Path::new("common/car_logs").exists()
            || std::path::Path::new("common").exists()
        {
            "common/car_logs".to_string()
        } else if std::path::Path::new("/boot/firmware").exists() {
            "/boot/firmware/car_logs".to_string()
        } else {
            "/boot/car_logs".to_string()
        }
    }

    pub fn new() -> Self {
        let log_dir = Self::get_log_dir();

        let logs_path = std::path::Path::new(&log_dir);
        if !logs_path.exists() {
            std::fs::create_dir_all(logs_path).expect("Failed to create car_logs directory");
        }

        Self {
            start_time: None,
            end_time: None,
            file: None,
            temp_path: String::new(),
            last_point_time: None,
            log_dir,
        }
    }

    pub fn cleanup_previous_runs() {
        let log_dir = Self::get_log_dir();
        let logs_path = std::path::Path::new(&log_dir);
        if !logs_path.exists() {
            return;
        }

        if let Ok(entries) = std::fs::read_dir(logs_path) {
            for entry in entries.flatten() {
                let path = entry.path();
                if let Some(file_name) = path.file_name().and_then(|n| n.to_str()) {
                    if file_name.ends_with("_inprogress.csv") {
                        let new_file_name = file_name.replace("_inprogress.csv", "_recovered.csv.zst");
                        let compressed_path = logs_path.join(new_file_name);
                        
                        let mut success = false;
                        if let Ok(mut temp_file) = File::open(&path) {
                            if let Ok(compressed_file) = File::create(&compressed_path) {
                                if let Ok(mut encoder) = zstd::stream::Encoder::new(compressed_file, 3) {
                                    if std::io::copy(&mut temp_file, &mut encoder).is_ok() {
                                        if encoder.finish().is_ok() {
                                            success = true;
                                            let _ = std::fs::remove_file(&path);
                                        }
                                    }
                                }
                            }
                        }
                        if !success {
                            // Fallback if compression fails, just rename to .csv
                            let fallback_name = file_name.replace("_inprogress.csv", "_recovered.csv");
                            let _ = std::fs::rename(&path, logs_path.join(fallback_name));
                        }
                    }
                }
            }
        }
    }

    pub fn record_point(
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

            let start_dt = chrono::Local.timestamp_opt(now as i64, 0).unwrap();
            let start_str = start_dt.format("%Y-%m-%d_%H-%M-%S");

            self.temp_path = format!("{}/{}_inprogress.csv", &self.log_dir, start_str);
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

    pub fn finish(mut self) {
        if let (Some(f), Some(start)) = (self.file.take(), self.start_time) {
            drop(f); // ensure it's flushed/closed

            let start_dt = chrono::Local.timestamp_opt(start as i64, 0).unwrap();
            let start_str = start_dt.format("%Y-%m-%d_%H-%M-%S");

            let compressed_path = format!("{}/{}_finished.csv.zst", &self.log_dir, start_str);
            
            let mut success = false;
            if let Ok(mut temp_file) = File::open(&self.temp_path) {
                if let Ok(compressed_file) = File::create(&compressed_path) {
                    if let Ok(mut encoder) = zstd::stream::Encoder::new(compressed_file, 3) {
                        if std::io::copy(&mut temp_file, &mut encoder).is_ok() {
                            if encoder.finish().is_ok() {
                                success = true;
                                let _ = std::fs::remove_file(&self.temp_path);
                            }
                        }
                    }
                }
            }
            
            if !success {
                // Fallback: Just rename it to a normal .csv if compression failed
                let new_path = format!("{}/{}_finished.csv", &self.log_dir, start_str);
                let _ = std::fs::rename(&self.temp_path, new_path);
            }
        }
    }
}
