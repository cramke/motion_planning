use rand::{Rng, rngs::ThreadRng};
use geo_types::Point;

/// Boundaries Limit the search space in 2D. Gives an upper and lower limit for the X- and Y-Coordinate.
/// Is implemented similar to a bounding box. That means as an upper / lower limit for the boundary axis.
/// Only 2D.
#[derive(Debug, Clone)]
pub struct Boundaries {
    pub x_lower: f64,
    pub x_upper: f64,
    pub y_lower: f64,
    pub y_upper: f64,
    rand: ThreadRng,
}

impl Boundaries {
    // Constructor for an Boundaries Object.
    pub fn new(x_lower: f64, x_upper: f64, y_lower: f64, y_upper: f64) -> Self {
        let rand: ThreadRng = rand::thread_rng();
        return Boundaries { x_lower, x_upper, y_lower, y_upper, rand };
    }

    /// Checks if node is inside the boundaries.
    /// Returns
    ///  - true: Node is inside space
    ///  - false: Node is outside space
    pub fn is_node_inside(&self, node: &Point) -> bool {
        if node.x() < self.x_lower {
            return false;
        }

        if node.x() > self.x_upper {
            return false;
        }

        if node.y() < self.y_lower {
            return false;
        }

        if node.y() > self.y_upper {
            return false;
        }

        return true;
    }

    /// Generates a random node, which is inside the boundary limits.
    /// Return
    ///  - Point: Has random coordinates.
    pub fn generate_random_configuration(&mut self) -> Point {
        let x: f64 = self.rand.gen_range(self.x_lower..self.x_upper);
        let y: f64 = self.rand.gen_range(self.y_lower..self.y_upper);
        let node: Point = Point::new(x, y);
        return node;
    }
}

mod tests {

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
        use geo_types::Point;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point = Point::new(0.5f64, 2.5f64);
        assert!(bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_x() {
        use crate::boundaries::Boundaries;
        use geo_types::Point;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point = Point::new(2.5f64, 2.5f64);
        assert!(!bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_y() {
        use crate::boundaries::Boundaries;
        use geo_types::Point;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point = Point::new(0.5f64, 0f64);
        assert!(!bounds.is_node_inside(&node));
    }

}