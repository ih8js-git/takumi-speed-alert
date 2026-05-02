use rstar::{primitives::{GeomWithData, Line}, RTree};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RoadData {
    pub speed_limit_mph: u8,
}

pub type RoadLine = Line<[f64; 2]>;
pub type RoadSegment = GeomWithData<RoadLine, RoadData>;
pub type RoadTree = RTree<RoadSegment>;
