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

#[cfg(test)]
mod tests {
    use super::Node2D;

    #[test]
    fn test_new_x() {
        let node = Node2D::new(2f64, 3f64);
        assert_eq!(2f64, node.x);
        assert_eq!(3f64, node.y);
        assert_eq!(0usize, node.idx);

    }
}