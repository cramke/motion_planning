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
        ((self.x - other.x) * (self.x - other.x)) + ((self.y - other.y) * (self.y - other.y))
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
