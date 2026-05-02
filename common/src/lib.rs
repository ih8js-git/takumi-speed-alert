use rstar::{
    RTree,
    primitives::{GeomWithData, Line},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RoadData {
    pub speed_limit_mph: u8,
    pub is_one_way: bool,
    pub bearing: u16, // compass bearing (0-359)
}

pub type RoadLine = Line<[f64; 2]>;
pub type RoadSegment = GeomWithData<RoadLine, RoadData>;
pub type RoadTree = RTree<RoadSegment>;

/// Calculates the difference between two angles in degrees, handling wrapping.
/// Returns the smallest difference, e.g. 350 degrees and 10 degrees is 20 degrees, not 340 degrees.
pub fn angle_diff(a: f64, b: f64) -> f64 {
    let diff = (a - b).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}
