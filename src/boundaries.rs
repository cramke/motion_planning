use crate::{core::Metric2D, space::Point};
use rand::{rngs::ThreadRng, Rng};

/// Boundaries Limit the search space in 2D. Gives an upper and lower limit for the X- and Y-Coordinate.
/// Is implemented similar to a bounding box. That means as an upper / lower limit for the boundary axis.
/// Only 2D.
#[derive(Debug, Clone)]
pub struct Boundaries<T: Metric2D> {
    pub x_lower: T,
    pub x_upper: T,
    pub y_lower: T,
    pub y_upper: T,
    rand: ThreadRng,
}

impl<T: Metric2D> Boundaries<T> {
    // Constructor for an Boundaries Object.
    pub fn new(x_lower: T, x_upper: T, y_lower: T, y_upper: T) -> Self {
        let rand: ThreadRng = rand::thread_rng();
        Boundaries {
            x_lower,
            x_upper,
            y_lower,
            y_upper,
            rand,
        }
    }

    /// Checks if node is inside the boundaries.
    /// Returns
    ///  - true: Node is inside space
    ///  - false: Node is outside space
    pub fn is_node_inside(&self, node: &Point<T>) -> bool {
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

        true
    }

    /// Generates a random node, which is inside the boundary limits.
    /// Return
    ///  - Point: Has random coordinates.
    pub fn generate_random_configuration(&mut self) -> Point<T> {
        let x: T = self.rand.gen_range(self.x_lower..self.x_upper);
        let y: T = self.rand.gen_range(self.y_lower..self.y_upper);
        Point { x, y }
    }
}

mod tests {

    #[test]
    fn test_boundaries_dummy_f64() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        assert_eq!(0f64, bounds.x_lower);
        assert_eq!(1f64, bounds.x_upper);
        assert_eq!(2f64, bounds.y_lower);
        assert_eq!(3f64, bounds.y_upper);
    }

    #[test]
    fn test_boundaries_dummy_f32() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries<f32> = Boundaries::new(0f32, 1f32, 2f32, 3f32);
        assert_eq!(0f32, bounds.x_lower);
        assert_eq!(1f32, bounds.x_upper);
        assert_eq!(2f32, bounds.y_lower);
        assert_eq!(3f32, bounds.y_upper);
    }

    #[test]
    fn test_boundaries_inside_true() {
        use crate::boundaries::Boundaries;
        use crate::space::Point;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point<f64> = Point {
            x: 0.5f64,
            y: 2.5f64,
        };
        assert!(bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_x() {
        use crate::boundaries::Boundaries;
        use crate::space::Point;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point<f64> = Point {
            x: 2.5f64,
            y: 2.5f64,
        };
        assert!(!bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_y() {
        use crate::boundaries::Boundaries;
        use crate::space::Point;

        let bounds: Boundaries<f64> = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point<f64> = Point { x: 0.5f64, y: 0f64 };
        assert!(!bounds.is_node_inside(&node));
    }
}
