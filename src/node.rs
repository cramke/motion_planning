#[derive(Debug, Copy, Clone)]
pub struct Node2D {
    pub x: f64,
    pub y: f64,
    pub idx: usize,
}

impl Node2D {
    pub fn new(x: f64, y: f64) -> Self {
        let idx: usize = 0;
        return Node2D { x: x, y: y, idx: idx }
    }
}