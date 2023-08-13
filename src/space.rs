use crate::core::Metric2D;

#[derive(Debug, Clone, Copy)]
pub struct Point<T: Metric2D> {
    pub x: T,
    pub y: T,
}

impl<T: Metric2D> PartialEq for Point<T> {
    fn eq(&self, other: &Self) -> bool {
        let eq_x: bool = (self.x - other.x).abs() < T::EPSILON; 
        let eq_y: bool = (self.y - other.y).abs() < T::EPSILON;
        eq_x && eq_y
    }
}

impl<T: Metric2D> Point<T> {
    pub fn to_wkt(&self) -> String {
        format!("POINT({} {})", self.x.to_string(), self.x.to_string())
    }

    pub fn euclidean_distance(&self, other: &Point<T>) -> T {
        ((self.x - other.x) * (self.x - other.x)) + ((self.y - other.y) * (self.y - other.y))
    }
}
