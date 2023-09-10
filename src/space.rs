use crate::types::SpaceContinuous;

/// Defines a struct called `Point` with two generic fields `x` and `y`.
#[derive(Debug, Clone, Copy)]
pub struct Point<T: SpaceContinuous> {
    x: T,
    y: T,
}

/// Implements the `PartialEq` trait for the `Point` struct. This trait allows for the equality comparison between two `Point` instances based on the difference between their x and y coordinates.
///
/// # Inputs
///
/// - `self`: A reference to the first `Point` instance.
/// - `other`: A reference to the second `Point` instance.
///
/// # Outputs
///
/// - `true` if the x and y coordinates of `self` and `other` are equal within the epsilon value, indicating that the two points are equal.
/// - `false` if the x and y coordinates of `self` and `other` are not equal within the epsilon value, indicating that the two points are not equal.
impl<T: SpaceContinuous> PartialEq for Point<T> {
    fn eq(&self, other: &Self) -> bool {
        let eq_x: bool = (self.x - other.x).abs() < T::EPSILON;
        let eq_y: bool = (self.y - other.y).abs() < T::EPSILON;
        eq_x && eq_y
    }
}

impl<T: SpaceContinuous> Point<T> {
    /// Creates a new `Point` instance with the given coordinates.
    ///
    /// # Parameters
    /// - `x`: The x-coordinate of the point.
    /// - `y`: The y-coordinate of the point.
    ///
    /// # Returns
    /// A new `Point` instance with the given coordinates.
    pub fn new(x: T, y: T) -> Self {
        Point { x, y }
    }

    /// Formats the point as a well-known text (WKT) string.
    ///
    /// # Returns
    /// The WKT representation of the point.
    pub fn to_wkt(&self) -> String {
        format!("POINT({} {})", self.x, self.x)
    }

    /// Calculates the Euclidean distance between the current point and another point.
    ///
    /// # Parameters
    /// - `other`: The other point to calculate the distance to.
    ///
    /// # Returns
    /// The Euclidean distance between the two points.
    pub fn euclidean_distance(&self, other: &Point<T>) -> T {
        let x_diff = self.get_x() - other.get_x();
        let y_diff = self.get_y() - other.get_y();
        T::sqrt(x_diff * x_diff + y_diff * y_diff)
    }

    /// Retrieves the x-coordinate of a Point instance.
    pub fn get_x(&self) -> T {
        self.x
    }

    /// Retrieves the y-coordinate of a Point instance.
    pub fn get_y(&self) -> T {
        self.y
    }
}

/// Implements the `Default` trait for the `Point` struct. This implementation provides a default value for the `Point` struct by setting the `x` and `y` coordinates to the default value defined in the `SpaceContinuous` trait for the generic type `T`.
///
/// # Inputs
///
/// - None
///
/// # Outputs
///
/// - A new `Point` instance with the `x` and `y` coordinates set to the default value.
impl<T: SpaceContinuous> Default for Point<T> {
    fn default() -> Self {
        Point {
            x: SpaceContinuous::DEFAULT,
            y: SpaceContinuous::DEFAULT,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::space::Point;

    // Test the calculation of the Euclidean distance between two points with positive coordinates.
    #[test]
    fn test_euclidean_distance_positive_coordinates() {
        // Create two points with positive coordinates
        let point1: Point<f64> = Point::new(3.0, 4.0);
        let point2: Point<f64> = Point::new(6.0, 8.0);

        // Calculate the Euclidean distance
        let distance = point1.euclidean_distance(&point2);

        // Assert that the distance is calculated correctly
        assert_eq!(distance, 5.0);
    }

    // Test the calculation of the Euclidean distance between two points with negative coordinates.
    #[test]
    fn test_euclidean_distance_negative_coordinates() {
        // Create two points with negative coordinates
        let point1: Point<f64> = Point::new(-2.0, -3.0);
        let point2: Point<f64> = Point::new(-5.0, -7.0);

        // Calculate the Euclidean distance
        let distance = point1.euclidean_distance(&point2);

        // Assert that the distance is calculated correctly
        assert_eq!(distance, 5.0);
    }

    // Test the calculation of the euclidean distance between two points when one of the coordinates is zero.
    #[test]
    fn test_euclidean_distance_with_zero_coordinate() {
        let point1: Point<f64> = Point::new(0.0, 0.0);
        let point2: Point<f64> = Point::new(3.0, 4.0);
        let distance: f64 = point1.euclidean_distance(&point2);
        assert_eq!(distance, 5.0);
    }

    // Test the calculation of the euclidean distance between two points with coordinates that are very far from each other.
    #[test]
    fn test_euclidean_distance_far_points() {
        // Create two points with very large coordinates
        let point1: Point<f64> = Point::new(1000000.0, 1000000.0);
        let point2: Point<f64> = Point::new(-1000000.0, -1000000.0);

        // Calculate the euclidean distance
        let distance: f64 = point1.euclidean_distance(&point2);

        // Assert that the distance is correct
        assert_eq!(distance, 2828427.12474619);
    }

    // Test the calculation of the euclidean distance between two points with coordinates that are very close to each other.
    #[test]
    fn test_euclidean_distance_close_coordinates() {
        // Create two points with very close coordinates
        let point1: Point<f64> = Point::new(0.0, 0.0);
        let point2: Point<f64> = Point::new(0.000001, 0.000001);

        // Calculate the euclidean distance
        let distance: f64 = point1.euclidean_distance(&point2);

        // Assert that the distance is close to zero
        assert!(distance < 0.00001);
    }

    // Test the calculation of the euclidean distance between two identical points
    #[test]
    fn test_euclidean_distance_identical_points() {
        let point1: Point<f64> = Point::new(0.0, 0.0);
        let point2: Point<f64> = Point::new(0.0, 0.0);
        let distance: f64 = point1.euclidean_distance(&point2);
        assert_eq!(distance, 0.0);
    }

    // Test the calculation of the Euclidean distance between two points with different x-coordinates but the same y-coordinate.
    #[test]
    fn test_euclidean_distance_different_x_same_y() {
        let point1: Point<f64> = Point::new(2.0, 3.0);
        let point2: Point<f64> = Point::new(5.0, 3.0);
        let distance: f64 = point1.euclidean_distance(&point2);
        assert_eq!(distance, 3.0);
    }

    // Test the calculation of the Euclidean distance between two points with different y-coordinates but the same x-coordinate.
    #[test]
    fn test_euclidean_distance_different_y_coordinates() {
        let point1: Point<f64> = Point::new(0.0, 0.0);
        let point2: Point<f64> = Point::new(0.0, 1.0);
        let distance: f64 = point1.euclidean_distance(&point2);
        assert_eq!(distance, 1.0);
    }
}
