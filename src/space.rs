use std::ops::{Sub, Add, Mul};
use num::Float;

#[derive(Debug, Clone, Copy)]
pub struct Point<T: PartialEq> 
{
    pub x: T,
    pub y: T
}

impl<T: PartialEq + Float> PartialEq for Point<T> {
    fn eq(&self, other: &Self) -> bool {
        let eq_x: bool = (self.x - other.x).abs() < T::epsilon();
        let eq_y: bool = (self.y - other.y).abs() < T::epsilon();
        eq_x && eq_y
    }
}

impl<T: ToString + Sub<Output = T> + Add<Output = T> + Mul<Output = T> + Copy + PartialEq> Point<T> 
{
    pub fn to_wkt(&self) -> String {
        format!("POINT({} {})", self.x.to_string(), self.x.to_string())
    }

    pub fn euclidean_distance(&self, other: &Point<T>) -> T {
        ((self.x - other.x)*(self.x - other.x)) + ((self.y - other.y)*(self.y - other.y))
    }
}