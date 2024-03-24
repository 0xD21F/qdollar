pub mod point;
pub mod point_cloud;
pub mod qdollar;
pub mod utils;

pub use point::Point;
pub use point_cloud::PointCloud;
pub use qdollar::QDollarRecognizer;
pub use qdollar::QDollarResult;

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(test)]
    use plotlib::page::Page;
    #[cfg(test)]
    use plotlib::repr::Plot;
    #[cfg(test)]
    use plotlib::style::{PointMarker, PointStyle};
    #[cfg(test)]
    use plotlib::view::ContinuousView;

    #[test]
    fn test_add_and_recognize_gesture() {
        let mut recognizer = QDollarRecognizer::new();

        let square_points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(0.0, 1.0, 1),
            Point::new(1.0, 1.0, 1),
            Point::new(1.0, 0.0, 1),
            Point::new(0.0, 0.0, 1),
        ];
        recognizer.add_gesture("square".to_string(), square_points.clone());

        let result = recognizer.recognize(&square_points);
        assert_eq!(result.name, "square");
        assert!(result.score > 0.9);
    }

    #[test]
    fn test_recognize_no_gestures() {
        let recognizer = QDollarRecognizer::new();

        let points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(0.0, 1.0, 1),
            Point::new(1.0, 1.0, 1),
            Point::new(1.0, 0.0, 1),
            Point::new(0.0, 0.0, 1),
        ];

        let result = recognizer.recognize(&points);
        assert_eq!(result.name, "No gestures registered");
        assert_eq!(result.score, 0.0);
    }

    #[test]
    fn test_recognize_multiple_gestures() {
        let mut recognizer = QDollarRecognizer::new();

        let square_points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(0.0, 1.0, 1),
            Point::new(1.0, 1.0, 1),
            Point::new(1.0, 0.0, 1),
            Point::new(0.0, 0.0, 1),
        ];
        recognizer.add_gesture("square".to_string(), square_points.clone());

        let triangle_points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(0.5, 1.0, 1),
            Point::new(1.0, 0.0, 1),
            Point::new(0.0, 0.0, 1),
        ];
        recognizer.add_gesture("triangle".to_string(), triangle_points.clone());

        let result_square = recognizer.recognize(&square_points);
        assert_eq!(result_square.name, "square");
        assert!(result_square.score > 0.9);

        let result_triangle = recognizer.recognize(&triangle_points);
        assert_eq!(result_triangle.name, "triangle");
        assert!(result_triangle.score > 0.9);
    }

    #[test]
    fn test_utils_resample() {
        // Square path
        let square_points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(0.0, 1.0, 1),
            Point::new(1.0, 1.0, 1),
            Point::new(1.0, 0.0, 1),
            Point::new(0.0, 0.0, 1),
        ];

        // Triangle path
        let triangle_points = vec![
            Point::new(0.0, 0.0, 2),
            Point::new(0.5, 1.0, 2),
            Point::new(1.0, 0.0, 2),
            Point::new(0.0, 0.0, 2),
        ];

        // Complex path
        let complex_points = vec![
            Point::new(0.0, 0.0, 3),
            Point::new(0.0, 1.0, 3),
            Point::new(0.5, 1.0, 3),
            Point::new(0.5, 0.5, 3),
            Point::new(1.0, 0.5, 3),
            Point::new(1.0, 0.0, 3),
            Point::new(0.0, 0.0, 3),
        ];

        let shapes = vec![
            ("square", square_points),
            ("triangle", triangle_points),
            ("complex", complex_points),
        ];

        for (shape_name, points) in shapes {
            let resampled_points = utils::resample(&points, utils::NUM_POINTS);

            // Plot the points and resampled points
            if let Err(e) = plot_points(
                &points,
                &resampled_points,
                &format!("{shape_name}_test_resample.svg"),
            ) {
                eprintln!("Failed to plot points: {}", e);
                panic!("Plotting failed");
            }

            // Check that the resampled points have the correct length
            assert_eq!(resampled_points.len(), utils::NUM_POINTS);

            // Check that the first point match the original first point
            assert_eq!(resampled_points[0], points[0]);
            // Check that the last point matches the original last point or is within a small threshold
            let last_point_threshold = 1e-6; // Adjust the threshold value as needed
            let last_resampled_point = &resampled_points[resampled_points.len() - 1];
            let last_original_point = &points[points.len() - 1];
            assert!(
                *last_resampled_point == *last_original_point ||
                (last_resampled_point.x - last_original_point.x).abs() < last_point_threshold &&
                (last_resampled_point.y - last_original_point.y).abs() < last_point_threshold,
                "Last resampled point is not equal to the last original point and differs by more than the threshold"
            );

            // Calculate the expected interval and tolerance based on the path length and shape
            let path_length = utils::path_length(&points);
            let expected_interval = path_length / (resampled_points.len() - 1) as f64;
            let max_deviation = expected_interval * 0.5; // Allow a 50% deviation

            let mut prev_point = resampled_points[0];
            for point in &resampled_points[1..] {
                let dist = utils::euclidean_distance(&prev_point, point);
                if (dist - expected_interval).abs() >= max_deviation {
                    println!("Tolerance exceeded: {:?} -> {:?}, dist: {}, expected_interval: {}, max_deviation: {}",
                            prev_point, point, dist, expected_interval, max_deviation);
                }
                assert!(
                    (dist - expected_interval).abs() < max_deviation,
                    "Distance between consecutive points should be within the allowed deviation"
                );
                prev_point = *point;
            }
        }
    }

    #[test]
    fn test_utils_scale() {
        let points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(10.0, 0.0, 1),
            Point::new(10.0, 10.0, 1),
            Point::new(0.0, 10.0, 1),
            Point::new(0.0, 0.0, 1),
        ];

        let scaled_points = utils::scale(&points);

        assert!(scaled_points.iter().all(|pt| pt.x >= 0.0 && pt.x <= 1.0));
        assert!(scaled_points.iter().all(|pt| pt.y >= 0.0 && pt.y <= 1.0));
    }

    #[test]
    fn test_utils_translate_to() {
        let points = vec![
            Point::new(0.0, 0.0, 1),
            Point::new(1.0, 0.0, 1),
            Point::new(1.0, 1.0, 1),
            Point::new(0.0, 1.0, 1),
            Point::new(0.0, 0.0, 1),
        ];

        let translated_points = utils::translate_to(&points, &utils::ORIGIN);

        let centroid = utils::centroid(&translated_points);
        assert!((centroid.x - utils::ORIGIN.x).abs() < 1e-6);
        assert!((centroid.y - utils::ORIGIN.y).abs() < 1e-6);
    }

    fn plot_points(
        points: &[Point],
        resampled_points: &[Point],
        filename: &String,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut data1 = Vec::new();
        for p in points {
            data1.push((p.x, p.y));
        }

        let mut data2 = Vec::new();
        for p in resampled_points {
            data2.push((p.x, p.y));
        }

        let view = ContinuousView::new()
            .add(
                Plot::new(data1)
                    .point_style(
                        PointStyle::new()
                            .marker(PointMarker::Cross)
                            .colour("#0000FF"),
                    )
                    .legend(String::from("Original Points")),
            )
            .add(
                Plot::new(data2)
                    .point_style(PointStyle::new().marker(PointMarker::Circle))
                    .legend(String::from("Resampled Points")),
            )
            .x_range(-1.0, 2.0)
            .y_range(-1.0, 2.0)
            .x_label("X")
            .y_label("Y");

        Page::single(&view).save(filename).map_err(|e| e.into())
    }
}
