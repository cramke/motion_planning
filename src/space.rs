use crate::types::SpaceContinuous;

#[derive(Debug, Clone, Copy)]
pub struct Point<T: SpaceContinuous> {
    pub x: T,
    pub y: T,
}

impl<T: SpaceContinuous> PartialEq for Point<T> {
    fn eq(&self, other: &Self) -> bool {
        let eq_x: bool = (self.x - other.x).abs() < T::EPSILON;
        let eq_y: bool = (self.y - other.y).abs() < T::EPSILON;
        eq_x && eq_y
    }
}

impl<T: SpaceContinuous> Point<T> {
    pub fn new(x: T, y: T) -> Self {
        Point { x, y }
    }

    pub fn to_wkt(&self) -> String {
        format!("POINT({} {})", self.x, self.x)
    }

    pub fn euclidean_distance(&self, other: &Point<T>) -> T {
        ((self.x - other.x) * (self.x - other.x)) + ((self.y - other.y) * (self.y - other.y))
    }
}

impl<T: SpaceContinuous> Default for Point<T> {
    fn default() -> Self {
        Point {
            x: SpaceContinuous::DEFAULT,
            y: SpaceContinuous::DEFAULT,
        }
    }
}
