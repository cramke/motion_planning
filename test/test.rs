#[cfg(test)]
mod test {
    use crate::Node2D;

    #[test]
    fn test_dummy() {
        let tree: f64 = 3f64;
        assert_eq!(tree, 3.0);
    }

    #[test]
    fn test_node() {
        let node: Node2D = Node2D {x: 0f64, y: 0f64, idx: 0};
        assert_eq!(node.x, 0f64);
        assert_eq!(node.y, 0f64);
    }

    #[test]
    fn test_node2d_new() {
        let node = Node2D::new(0f64, 0f64);
        assert_eq!(node.x, 0f64);
        assert_eq!(node.y, 0f64);
        assert_eq!(node.idx, 0usize);
    }
}