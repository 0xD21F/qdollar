#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub id: u32,
    pub int_x: usize,
    pub int_y: usize,
}

impl Point {
    pub fn new(x: f64, y: f64, id: u32) -> Self {
        Point {
            x,
            y,
            id,
            int_x: 0,
            int_y: 0,
        }
    }
}
