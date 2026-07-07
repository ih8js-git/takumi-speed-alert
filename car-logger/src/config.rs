use serde::{Deserialize, Serialize};

pub const CONFIG_PATH: &str = "config.json";

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    pub heading_tolerance_degrees: f64,
    pub speed_limit_tolerance_mph: f64,
    pub speeding_duration_before_beep_seconds: f64,
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
