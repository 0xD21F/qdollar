use crate::{utils, Point, PointCloud};
use crate::error::QDollarError;

pub struct QDollarRecognizer {
    point_clouds: Vec<PointCloud>,
}

#[derive(Debug)]
pub struct QDollarResult {
    pub name: String,
    pub score: f64,
    pub time: u128,
}

impl QDollarRecognizer {
    pub fn new() -> Self {
        QDollarRecognizer {
            point_clouds: Vec::new(),
        }
    }

    pub fn recognize(&self, points: &[Point]) -> Result<QDollarResult, QDollarError> {
        let now = std::time::Instant::now();

        if self.point_clouds.is_empty() {
            return Err(QDollarError::NoRegisteredGestures);
        }
    
        let candidate = PointCloud::new(String::new(), points.to_vec());

        let mut best_distance = f64::INFINITY;
        let mut best_template = 0;

        for (i, template) in self.point_clouds.iter().enumerate() {
            let dist = utils::cloud_match(&candidate, template, best_distance);
            if dist < best_distance {
                best_distance = dist;
                best_template = i;
            }
        }

        let score = if best_distance > 1.0 {
            1.0 / best_distance
        } else {
            1.0
        };

        Ok(QDollarResult {
            name: self.point_clouds[best_template].name.clone(),
            score,
            time: now.elapsed().as_millis(),
        })
    }

    pub fn add_gesture(&mut self, name: String, points: Vec<Point>) -> usize {
        self.point_clouds.push(PointCloud::new(name, points));
        self.point_clouds.len()
    }

    pub fn delete_user_gestures(&mut self) -> usize {
        self.point_clouds.clear();
        self.point_clouds.len()
    }
}
