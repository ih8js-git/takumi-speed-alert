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
    pub fn new() -> Self {
        let log_dir = if std::path::Path::new("common/car_logs").exists()
            || std::path::Path::new("common").exists()
        {
            "common/car_logs".to_string()
        } else if std::path::Path::new("/boot/firmware").exists() {
            "/boot/firmware/car_logs".to_string()
        } else {
            "/boot/car_logs".to_string()
        };

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
        if let (Some(f), Some(start), Some(end)) =
            (self.file.take(), self.start_time, self.end_time)
        {
            drop(f); // ensure it's flushed/closed

            let start_dt = chrono::Local.timestamp_opt(start as i64, 0).unwrap();
            let end_dt = chrono::Local.timestamp_opt(end as i64, 0).unwrap();

            let start_str = start_dt.format("%Y-%m-%d_%H-%M-%S");
            let end_str = end_dt.format("%Y-%m-%d_%H-%M-%S");

            let new_path = format!("{}/{}_to_{}.csv", &self.log_dir, start_str, end_str);
            let _ = std::fs::rename(&self.temp_path, new_path);
        }
    }
}
