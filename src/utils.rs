use crate::{Point, PointCloud};

pub const NUM_POINT_CLOUDS: usize = 16;
pub const NUM_POINTS: usize = 32;
pub const ORIGIN: Point = Point {
    x: 0.0,
    y: 0.0,
    id: 0,
    int_x: 0,
    int_y: 0,
};
pub const MAX_INT_COORD: usize = 1024;
pub const LUT_SIZE: usize = 64;
pub const LUT_SCALE_FACTOR: f64 = MAX_INT_COORD as f64 / LUT_SIZE as f64;

pub fn resample(points: &[Point], n: usize) -> Vec<Point> {
    let path_length = path_length(points);
    let interval = path_length / (n - 1) as f64;
    let mut new_points = vec![points[0]];
    let mut dist = 0.0;
    let mut i = 1;
    let mut points_vec = points.to_vec();
    while i < points_vec.len() {
        if points_vec[i].id == points_vec[i - 1].id {
            let d = euclidean_distance(&points_vec[i - 1], &points_vec[i]);
            if dist + d >= interval {
                let ratio = (interval - dist) / d;
                let qx = points_vec[i - 1].x + ratio * (points_vec[i].x - points_vec[i - 1].x);
                let qy = points_vec[i - 1].y + ratio * (points_vec[i].y - points_vec[i - 1].y);
                let q = Point::new(qx, qy, points_vec[i].id);
                new_points.push(q);
                points_vec.insert(i, q);
                dist = 0.0;
            } else {
                dist += d;
            }
        }
        i += 1;
    }
    if new_points.len() == n - 1 {
        new_points.push(*points_vec.last().unwrap());
    }
    new_points
}

pub fn scale(points: &[Point]) -> Vec<Point> {
    let (min_x, max_x, min_y, max_y) = points.iter().fold(
        (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ),
        |(min_x, max_x, min_y, max_y), pt| {
            (
                min_x.min(pt.x),
                max_x.max(pt.x),
                min_y.min(pt.y),
                max_y.max(pt.y),
            )
        },
    );

    let size = (max_x - min_x).max(max_y - min_y);

    points
        .iter()
        .map(|pt| Point::new((pt.x - min_x) / size, (pt.y - min_y) / size, pt.id))
        .collect()
}

pub fn translate_to(points: &[Point], pt: &Point) -> Vec<Point> {
    let centroid = centroid(points);
    points
        .iter()
        .map(|p| Point::new(p.x + pt.x - centroid.x, p.y + pt.y - centroid.y, p.id))
        .collect()
}

pub fn make_int_coords(points: &[Point]) -> Vec<Point> {
    points
        .iter()
        .map(|pt| {
            let mut p = *pt;
            p.int_x = ((pt.x + 1.0) / 2.0 * (MAX_INT_COORD - 1) as f64).round() as usize;
            p.int_y = ((pt.y + 1.0) / 2.0 * (MAX_INT_COORD - 1) as f64).round() as usize;
            p
        })
        .collect()
}

pub fn compute_lut(points: &[Point]) -> Vec<Vec<usize>> {
    let mut lut = vec![vec![0; LUT_SIZE]; LUT_SIZE];

    for x in 0..LUT_SIZE {
        for y in 0..LUT_SIZE {
            let mut min_distance = f64::INFINITY;
            let mut min_index = 0;

            for (i, pt) in points.iter().enumerate() {
                let row = (pt.int_x as f64 / LUT_SCALE_FACTOR).round() as usize;
                let col = (pt.int_y as f64 / LUT_SCALE_FACTOR).round() as usize;
                let dist = (row as isize - x as isize).pow(2) + (col as isize - y as isize).pow(2);
                if (dist as f64) < min_distance {
                    min_distance = dist as f64;
                    min_index = i;
                }
            }

            lut[x][y] = min_index;
        }
    }

    lut
}

pub fn cloud_match(candidate: &PointCloud, template: &PointCloud, min_so_far: f64) -> f64 {
    let n = candidate.points.len();
    let step = (n as f64).sqrt().floor() as usize;

    let lb1 = compute_lower_bound(&candidate.points, &template.points, step, &template.lut);
    let lb2 = compute_lower_bound(&template.points, &candidate.points, step, &candidate.lut);

    let mut min_val = min_so_far;

    for (i, &lb) in lb1.iter().enumerate().step_by(step) {
        if lb < min_val {
            min_val = min_val.min(cloud_distance(
                &candidate.points,
                &template.points,
                i,
                min_val,
            ));
        }
    }

    for (i, &lb) in lb2.iter().enumerate().step_by(step) {
        if lb < min_val {
            min_val = min_val.min(cloud_distance(
                &template.points,
                &candidate.points,
                i,
                min_val,
            ));
        }
    }

    min_val
}

fn cloud_distance(pts1: &[Point], pts2: &[Point], start: usize, min_so_far: f64) -> f64 {
    let n = pts1.len();
    let mut unmatched = (0..pts2.len()).collect::<Vec<_>>();
    let mut i = start;
    let mut weight = n;
    let mut sum = 0.0;

    loop {
        let mut min_dist = f64::INFINITY;
        let mut min_index = 0;

        for (j, &k) in unmatched.iter().enumerate() {
            let d = sqr_euclidean_distance(&pts1[i], &pts2[k]);
            if d < min_dist {
                min_dist = d;
                min_index = j;
            }
        }

        let _matched_index = unmatched.remove(min_index);
        sum += weight as f64 * min_dist;

        if sum >= min_so_far {
            return sum;
        }

        weight -= 1;
        i = (i + 1) % n;

        if i == start || unmatched.is_empty() {
            break;
        }
    }

    sum
}

fn compute_lower_bound(
    pts1: &[Point],
    pts2: &[Point],
    step: usize,
    lut: &[Vec<usize>],
) -> Vec<f64> {
    let n = pts1.len();
    let mut lb = vec![0.0; n / step + 1];
    let mut sat = vec![0.0; n];

    for i in 0..n {
        let x = (pts1[i].int_x as f64 / LUT_SCALE_FACTOR).round() as usize;
        let y = (pts1[i].int_y as f64 / LUT_SCALE_FACTOR).round() as usize;
        let index = lut[x][y];
        let d = sqr_euclidean_distance(&pts1[i], &pts2[index]);
        sat[i] = if i == 0 { d } else { sat[i - 1] + d };
        lb[0] += (n - i) as f64 * d;
    }

    for (i, j) in (step..n).step_by(step).zip(1..) {
        lb[j] = lb[0] + i as f64 * sat[n - 1] - n as f64 * sat[i - 1];
    }

    lb
}

pub fn path_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .filter(|pair| pair[0].id == pair[1].id)
        .map(|pair| euclidean_distance(&pair[0], &pair[1]))
        .sum()
}

pub fn centroid(points: &[Point]) -> Point {
    let (x, y) = points
        .iter()
        .fold((0.0, 0.0), |(x, y), pt| (x + pt.x, y + pt.y));

    Point::new(x / points.len() as f64, y / points.len() as f64, 0)
}

pub fn euclidean_distance(pt1: &Point, pt2: &Point) -> f64 {
    sqr_euclidean_distance(pt1, pt2).sqrt()
}

pub fn sqr_euclidean_distance(pt1: &Point, pt2: &Point) -> f64 {
    let dx = pt2.x - pt1.x;
    let dy = pt2.y - pt1.y;
    dx * dx + dy * dy
}
