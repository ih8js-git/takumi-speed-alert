#[cfg(feature = "rtree")]
pub use rtree_types::*;

#[cfg(feature = "rtree")]
mod rtree_types {
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
}

// --- rkyv zero-copy types for the spatial grid ---

use rkyv::{Archive, Deserialize, Serialize};

/// A single road segment with its geometry and metadata.
#[derive(Clone, Debug, Archive, Serialize, Deserialize)]
pub struct GridRoadSegment {
    pub p1: [f64; 2], // [lon, lat] start
    pub p2: [f64; 2], // [lon, lat] end
    pub speed_limit_mph: u8,
    pub is_one_way: bool,
    pub bearing: u16, // compass bearing (0-359)
}

/// A flat spatial grid that can be queried directly from mmap via rkyv zero-copy.
///
/// The grid divides a bounding box into uniform cells. Each cell contains
/// a list of road segments that intersect it. Queries search a 3×3
/// neighborhood of cells around the target point.
#[derive(Archive, Serialize, Deserialize)]
pub struct SpatialGrid {
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
    pub cell_size: f64,
    pub cols: u32,
    pub rows: u32,
    /// Flat 2D array of cells, indexed as `row * cols + col`.
    /// Each cell is a Vec of road segments that overlap that cell.
    pub cells: Vec<Vec<GridRoadSegment>>,
}

impl SpatialGrid {
    /// Default cell size in degrees (~500m at mid-latitudes).
    pub const DEFAULT_CELL_SIZE: f64 = 0.005;

    /// Build a spatial grid from a list of road segments.
    pub fn from_segments(segments: &[GridRoadSegment], cell_size: f64) -> Self {
        if segments.is_empty() {
            return Self {
                min_lon: 0.0,
                min_lat: 0.0,
                max_lon: 0.0,
                max_lat: 0.0,
                cell_size,
                cols: 0,
                rows: 0,
                cells: Vec::new(),
            };
        }

        // Compute bounding box with a small margin
        let margin = cell_size;
        let mut min_lon = f64::MAX;
        let mut min_lat = f64::MAX;
        let mut max_lon = f64::MIN;
        let mut max_lat = f64::MIN;

        for seg in segments {
            for p in &[seg.p1, seg.p2] {
                min_lon = min_lon.min(p[0]);
                min_lat = min_lat.min(p[1]);
                max_lon = max_lon.max(p[0]);
                max_lat = max_lat.max(p[1]);
            }
        }

        min_lon -= margin;
        min_lat -= margin;
        max_lon += margin;
        max_lat += margin;

        let cols = ((max_lon - min_lon) / cell_size).ceil() as u32;
        let rows = ((max_lat - min_lat) / cell_size).ceil() as u32;
        let total = (cols as usize) * (rows as usize);

        let mut cells: Vec<Vec<GridRoadSegment>> = Vec::with_capacity(total);
        cells.resize_with(total, Vec::new);

        for seg in segments {
            // Find the range of cells this segment's bounding box overlaps
            let seg_min_lon = seg.p1[0].min(seg.p2[0]);
            let seg_max_lon = seg.p1[0].max(seg.p2[0]);
            let seg_min_lat = seg.p1[1].min(seg.p2[1]);
            let seg_max_lat = seg.p1[1].max(seg.p2[1]);

            let col_lo = ((seg_min_lon - min_lon) / cell_size).floor().max(0.0) as u32;
            let col_hi = ((seg_max_lon - min_lon) / cell_size).floor().min((cols - 1) as f64) as u32;
            let row_lo = ((seg_min_lat - min_lat) / cell_size).floor().max(0.0) as u32;
            let row_hi = ((seg_max_lat - min_lat) / cell_size).floor().min((rows - 1) as f64) as u32;

            for row in row_lo..=row_hi {
                for col in col_lo..=col_hi {
                    let idx = (row as usize) * (cols as usize) + (col as usize);
                    cells[idx].push(seg.clone());
                }
            }
        }

        Self {
            min_lon,
            min_lat,
            max_lon,
            max_lat,
            cell_size,
            cols,
            rows,
            cells,
        }
    }
}

// --- Query methods on the archived (zero-copy) grid ---

impl ArchivedSpatialGrid {
    /// Convert a (lon, lat) point to grid (col, row), or None if out of bounds.
    fn cell_coords(&self, lon: f64, lat: f64) -> Option<(u32, u32)> {
        let cell_size: f64 = self.cell_size.into();
        let min_lon: f64 = self.min_lon.into();
        let min_lat: f64 = self.min_lat.into();
        let cols: u32 = self.cols.into();
        let rows: u32 = self.rows.into();

        let col = ((lon - min_lon) / cell_size).floor() as i64;
        let row = ((lat - min_lat) / cell_size).floor() as i64;

        if col < 0 || row < 0 || col >= cols as i64 || row >= rows as i64 {
            None
        } else {
            Some((col as u32, row as u32))
        }
    }

    /// Find the nearest road to the given point, optionally filtering by heading.
    ///
    /// Returns `(speed_limit_mph, distance_in_degrees)` or `None` if no road is nearby.
    pub fn nearest_road(
        &self,
        point: &[f64; 2],
        heading: Option<f64>,
        heading_tolerance: f64,
    ) -> (Option<u8>, Option<f64>) {
        let cols: u32 = self.cols.into();
        let rows: u32 = self.rows.into();

        let Some((center_col, center_row)) = self.cell_coords(point[0], point[1]) else {
            return (None, None);
        };

        let mut best_speed: Option<u8> = None;
        let mut best_dist = f64::MAX;

        // Search 3×3 neighborhood
        let row_lo = center_row.saturating_sub(1);
        let row_hi = (center_row + 1).min(rows - 1);
        let col_lo = center_col.saturating_sub(1);
        let col_hi = (center_col + 1).min(cols - 1);

        for row in row_lo..=row_hi {
            for col in col_lo..=col_hi {
                let idx = (row as usize) * (cols as usize) + (col as usize);
                let cell = &self.cells[idx];

                for seg in cell.iter() {
                    let p1 = [f64::from(seg.p1[0]), f64::from(seg.p1[1])];
                    let p2 = [f64::from(seg.p2[0]), f64::from(seg.p2[1])];
                    let dist = point_to_segment_distance(point, &p1, &p2);

                    if dist > 0.05 {
                        continue; // Too far away
                    }

                    let road_bearing: f64 = u16::from(seg.bearing) as f64;
                    let is_one_way: bool = seg.is_one_way.into();

                    if let Some(h) = heading {
                        let diff = angle_diff(h, road_bearing);
                        let mut matched = diff <= heading_tolerance;

                        if !matched && !is_one_way {
                            let reverse_diff = angle_diff(h, road_bearing + 180.0);
                            if reverse_diff <= heading_tolerance {
                                matched = true;
                            }
                        }

                        if !matched {
                            continue;
                        }
                    }

                    if dist < best_dist {
                        best_dist = dist;
                        best_speed = Some(seg.speed_limit_mph.into());
                    }
                }
            }
        }

        if best_speed.is_some() {
            (best_speed, Some(best_dist))
        } else {
            (None, None)
        }
    }
}

// --- Geometry helpers ---

/// Calculates the difference between two angles in degrees, handling wrapping.
/// Returns the smallest difference, e.g. 350 degrees and 10 degrees is 20 degrees, not 340 degrees.
pub fn angle_diff(a: f64, b: f64) -> f64 {
    let diff = (a - b).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}

/// Computes the minimum distance from a point to a line segment (in the same coordinate units).
fn point_to_segment_distance(point: &[f64; 2], p1: &[f64; 2], p2: &[f64; 2]) -> f64 {
    let dx = p2[0] - p1[0];
    let dy = p2[1] - p1[1];
    let len_sq = dx * dx + dy * dy;

    if len_sq == 0.0 {
        // Degenerate segment (point)
        let ex = point[0] - p1[0];
        let ey = point[1] - p1[1];
        return (ex * ex + ey * ey).sqrt();
    }

    // Parameter t for projection of point onto the infinite line through p1-p2
    let t = ((point[0] - p1[0]) * dx + (point[1] - p1[1]) * dy) / len_sq;
    let t_clamped = t.clamp(0.0, 1.0);

    let proj_x = p1[0] + t_clamped * dx;
    let proj_y = p1[1] + t_clamped * dy;

    let ex = point[0] - proj_x;
    let ey = point[1] - proj_y;
    (ex * ex + ey * ey).sqrt()
}
