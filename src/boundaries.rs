use crate::space::Point;
use rand::{rngs::ThreadRng, Rng};

/// Boundaries Limit the search space in 2D. Gives an upper and lower limit for the X- and Y-Coordinate.
/// Is implemented similar to a bounding box. That means as an upper / lower limit for the boundary axis.
/// Only 2D.
#[derive(Debug, Clone)]
pub struct Boundaries {
    x_lower: f64,
    x_upper: f64,
    y_lower: f64,
    y_upper: f64,
    rand: ThreadRng,
}

impl Boundaries {
    pub fn get_x_lower(&self) -> f64 {
        self.x_lower
    }

    pub fn set_x_lower(&mut self, value: f64) {
        self.x_lower = value;
    }

    pub fn get_x_upper(&self) -> f64 {
        self.x_upper
    }

    pub fn set_x_upper(&mut self, value: f64) {
        self.x_upper = value;
    }

    pub fn get_y_lower(&self) -> f64 {
        self.y_lower
    }

    pub fn set_y_lower(&mut self, value: f64) {
        self.y_lower = value;
    }

    pub fn get_y_upper(&self) -> f64 {
        self.y_upper
    }

    pub fn set_y_upper(&mut self, value: f64) {
        self.y_upper = value;
    }
}

impl Boundaries {
    // Constructor for an Boundaries Object.
    pub fn new(x_lower: f64, x_upper: f64, y_lower: f64, y_upper: f64) -> Self {
        let rand = rand::thread_rng();
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
    pub fn is_node_inside(&self, node: &Point) -> bool {
        if node.get_x() < self.x_lower {
            return false;
        }

        if node.get_x() > self.x_upper {
            return false;
        }

        if node.get_y() < self.y_lower {
            return false;
        }

        if node.get_y() > self.y_upper {
            return false;
        }

        true
    }

    /// Generates a random node, which is inside the boundary limits.
    /// Return
    ///  - Point: Has random coordinates.
    pub fn generate_random_configuration(&mut self) -> Point {
        let x: f64 = self.rand.gen_range(self.x_lower..self.x_upper);
        let y: f64 = self.rand.gen_range(self.y_lower..self.y_upper);
        Point::new(x, y)
    }
}

/// Implements the `Default` trait for the `Boundaries` struct.
///
/// This trait provides a default constructor for creating a `Boundaries` object with default values for the lower and upper limits of the X and Y coordinates.
impl Default for Boundaries {
    fn default() -> Self {
        Boundaries::new(f64::default(), f64::MAX, f64::default(), f64::MAX)
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_boundaries_dummy_f64() {
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
        use crate::space::Point;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point = Point::new(0.5f64, 2.5f64);
        assert!(bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_x() {
        use crate::boundaries::Boundaries;
        use crate::space::Point;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point = Point::new(2.5f64, 2.5f64);
        assert!(!bounds.is_node_inside(&node));
    }

    #[test]
    fn test_boundaries_inside_false_y() {
        use crate::boundaries::Boundaries;
        use crate::space::Point;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        let node: Point = Point::new(0.5f64, 0f64);
        assert!(!bounds.is_node_inside(&node));
    }

    // Test that the function 'test_boundaries_dummy_f64' returns a Boundaries object with the correct x and y limits.
    #[test]
    fn test_boundaries_dummy_f64_returns_correct_limits() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0f64, 1f64, 2f64, 3f64);
        assert_eq!(0f64, bounds.x_lower);
        assert_eq!(1f64, bounds.x_upper);
        assert_eq!(2f64, bounds.y_lower);
        assert_eq!(3f64, bounds.y_upper);
    }

    // Test that the function returns a Boundaries object with the minimum possible values for all limits.
    #[test]
    fn test_boundaries_minimum_values() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(f64::MIN, f64::MIN, f64::MIN, f64::MIN);
        assert_eq!(f64::MIN, bounds.x_lower);
        assert_eq!(f64::MIN, bounds.x_upper);
        assert_eq!(f64::MIN, bounds.y_lower);
        assert_eq!(f64::MIN, bounds.y_upper);
    }

    // Test that the function returns a Boundaries object with the maximum possible values for all limits.
    #[test]
    fn test_boundaries_maximum_values() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(f64::MAX, f64::MAX, f64::MAX, f64::MAX);
        assert_eq!(f64::MAX, bounds.x_lower);
        assert_eq!(f64::MAX, bounds.x_upper);
        assert_eq!(f64::MAX, bounds.y_lower);
        assert_eq!(f64::MAX, bounds.y_upper);
    }

    // Test the behavior of the 'test_boundaries_dummy_f64' function when given negative input values
    #[test]
    fn test_boundaries_negative_input() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(-1f64, -2f64, -3f64, -4f64);
        assert_eq!(-1f64, bounds.x_lower);
        assert_eq!(-2f64, bounds.x_upper);
        assert_eq!(-3f64, bounds.y_lower);
        assert_eq!(-4f64, bounds.y_upper);
    }

    // Test the behavior of the 'test_boundaries_dummy_f64' function when given non-integer input values
    #[test]
    fn test_boundaries_dummy_f64_non_integer_input() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(0.5f64, 1.5f64, 2.5f64, 3.5f64);
        assert_eq!(0.5f64, bounds.x_lower);
        assert_eq!(1.5f64, bounds.x_upper);
        assert_eq!(2.5f64, bounds.y_lower);
        assert_eq!(3.5f64, bounds.y_upper);
    }

    // Test that the function returns the expected values when given input values that are not in sequential order.
    #[test]
    fn test_boundaries_dummy_f64_input_not_in_sequential_order() {
        use crate::boundaries::Boundaries;

        let bounds: Boundaries = Boundaries::new(1f64, 0f64, 3f64, 2f64);
        assert_eq!(1f64, bounds.x_lower);
        assert_eq!(0f64, bounds.x_upper);
        assert_eq!(3f64, bounds.y_lower);
        assert_eq!(2f64, bounds.y_upper);
    }
}
