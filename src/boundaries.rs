use crate::node::Node2D;

#[derive(Debug, Copy, Clone)]
pub struct Boundaries {
    pub x_lower: f64,
    pub x_upper: f64,
    pub y_lower: f64,
    pub y_upper: f64,
}

impl Boundaries {
    pub fn new(x_lower: f64, x_upper: f64, y_lower: f64, y_upper: f64) -> Self {
        return Boundaries { x_lower, x_upper, y_lower, y_upper };
    }

    pub fn is_node_inside(&self, node: &Node2D) -> bool {
        if node.x < self.x_lower {
            return false;
        }

        if node.x > self.x_upper {
            return false;
        }

        if node.y < self.y_lower {
            return false;
        }

        if node.y > self.y_upper {
            return false;
        }

        return true;
    }
}

mod tests {
    use crate::node::Node2D;


    #[test]
    fn test_boundaries_dummy() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        assert_eq!(0f64, bounds.x_lower);
        assert_eq!(1f64, bounds.x_upper);
        assert_eq!(2f64, bounds.y_lower);
        assert_eq!(3f64, bounds.y_upper);
    }

    #[test]
    fn test_boundaries_inside_true() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Node2D = Node2D::new(0.5f64, 2.5f64);
        assert!(bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_x() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Node2D = Node2D::new(2.5f64, 2.5f64);
        assert!(!bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_y() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Node2D = Node2D::new(0.5f64, 0f64);
        assert!(!bounds.is_node_inside(&node));
    }

}