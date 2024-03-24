use crate::{utils, Point};

pub struct PointCloud {
    pub name: String,
    pub points: Vec<Point>,
    pub lut: Vec<Vec<usize>>,
}

impl PointCloud {
    pub fn new(name: String, points: Vec<Point>) -> Self {
        let points = utils::resample(&points, utils::NUM_POINTS);
        let points = utils::scale(&points);
        let points = utils::translate_to(&points, &utils::ORIGIN);
        let points = utils::make_int_coords(&points);
        let lut = utils::compute_lut(&points);

        PointCloud { name, points, lut }
    }
}
